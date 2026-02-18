/**
 * @file StateEstimator.hpp
 * @brief 现代C++ 状态估计器
 */

#ifndef STATE_ESTIMATOR_HPP
#define STATE_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include "Filter/kalman_filter.hpp"

namespace controller {

/**
* @brief 状态估计器类
* 
* 功能:
* 1. 腿部状态估计（角度、腿长及其导数）
* 2. 支撑力估计
* 3. 速度位置估计（卡尔曼滤波）
* 4. 离地检测
*/
class StateEstimator {
public:
    /**
    * @brief 状态估计器配置
    */
    struct Config {
        
        float dt = 0.003f;  // 采样周期

        // 卡尔曼滤波器参数
        struct {
            std::array<float, 2> Q = {0.5f, 0.5f};     // 过程噪声
            std::array<float, 2> R = {100.0f, 100.0f}; // 测量噪声
        } kalman_filter_param;
        
        // 离地检测阈值
        struct {
            float normal_force_min = 100.0f;
            float normal_force_max = 150.0f;
            float airborne_threshold = 80.0f;
            float overload_threshold = 200.0f;
            int stable_count_threshold = 20;
        } ground_detection;
    };
    
    /**
    * @brief 单腿状态
    */
    struct LegState {
        std::array<float, 3> alpha{0.0f};   // alpha历史 [current, -1, -2]
        std::array<float, 3> theta{0.0f};   // theta历史
        std::array<float, 3> L0{0.0f};      // 腿长历史
        
        float dot_alpha = 0.0f;
        float ddot_alpha = 0.0f;
        float dot_theta = 0.0f;
        float ddot_theta = 0.0f;
        float dot_L0 = 0.0f;
        float ddot_L0 = 0.0f;
    };
    
    /**
    * @brief 支撑力估计
    */
    struct ForceEstimate {
        float ddot_zb = 0.0f;                    // 机体竖直加速度
        std::array<float, 2> ddot_zw{0.0f};      // 轮子竖直加速度
        std::array<float, 2> Fn{0.0f};           // 支撑力

        bool ground_signal_ = false;             // 离地检测信号

    };
    
    /**
    * @brief 速度位置估计
    */
    struct VelocityEstimate {
        std::array<float, 2> w_ecd{0.0f};  // 轮子编码器角速度
        float b_acc = 0.0f;                // 机体加速度
        float b_vel = 0.0f;                // 机体角速度
        std::array<float, 2> dot_xw{0.0f}; // 轮子线速度
        float aver_vel = 0.0f;             // 平均速度
        float x_filter = 0.0f;             // 滤波后位置
        float v_filter = 0.0f;             // 滤波后速度

        std::array<bool,2> slip_signal_ = {false, false};// 打滑检测信号
    };
    
    /**
    * @brief 构造函数
    */
    StateEstimator() : StateEstimator(Config()) {}
    
    /**
    * @brief 带配置的构造函数
    */
    explicit StateEstimator(const Config& config) 
        : config_(config)
        , kalman_filter_(2, 0, 2)  // 2状态, 0控制, 2测量
    {
        initializeKalmanFilter();
    }
    
    /**
    * @brief 设置配置
    */
    void setConfig(const Config& config) {
        config_ = config;
        initializeKalmanFilter();
    }
    
    /**
    * @brief 获取配置
    */
    const Config& getConfig() const { return config_; }
    
    /**
    * @brief 更新腿部状态
    * @param alpha 左右腿alpha角 [left, right]
    * @param theta 左右腿theta角 [left, right]
    * @param L0 左右腿腿长 [left, right]
    */
    void update(const std::array<float, 2>& alpha,
                const std::array<float, 2>& theta,
                const std::array<float, 2>& L0) {

        const float dt = config_.dt;
        
        for (int i = 0; i < 2; ++i) {
            // 更新历史
            leg_states_[i].alpha[2] = leg_states_[i].alpha[1];
            leg_states_[i].alpha[1] = leg_states_[i].alpha[0];
            leg_states_[i].alpha[0] = alpha[i];
            
            leg_states_[i].theta[2] = leg_states_[i].theta[1];
            leg_states_[i].theta[1] = leg_states_[i].theta[0];
            leg_states_[i].theta[0] = theta[i];
            
            leg_states_[i].L0[2] = leg_states_[i].L0[1];
            leg_states_[i].L0[1] = leg_states_[i].L0[0];
            leg_states_[i].L0[0] = L0[i];
            
            // 计算一阶导数（向后差分）
            leg_states_[i].dot_alpha = 
                (leg_states_[i].alpha[0] - leg_states_[i].alpha[1]) / dt;
            leg_states_[i].dot_theta = 
                (leg_states_[i].theta[0] - leg_states_[i].theta[1]) / dt;
            leg_states_[i].dot_L0 = 
                (leg_states_[i].L0[0] - leg_states_[i].L0[1]) / dt;
            
            // 计算二阶导数（中心差分）
            leg_states_[i].ddot_alpha = 
                (leg_states_[i].alpha[0] - 2.0f * leg_states_[i].alpha[1] + 
                leg_states_[i].alpha[2]) / (dt * dt);
            leg_states_[i].ddot_theta = 
                (leg_states_[i].theta[0] - 2.0f * leg_states_[i].theta[1] + 
                leg_states_[i].theta[2]) / (dt * dt);
            leg_states_[i].ddot_L0 = 
                (leg_states_[i].L0[0] - 2.0f * leg_states_[i].L0[1] + 
                leg_states_[i].L0[2]) / (dt * dt);
        }
    }
    
    /**
    * @brief 估计支撑力
    * @param F 沿腿方向的力 [left, right]
    * @param Tp 绕机体转轴的力矩 [left, right]
    * @param ddot_zb 机体竖直加速度
    */
    void estimateForce(const std::array<float, 2>& F,
                    const std::array<float, 2>& Tp,
                    float ddot_zb,
                    float wheel_mass) {
        
        force_estimate_.ddot_zb = ddot_zb;
        
        for (int i = 0; i < 2; ++i) {
            const auto& leg = leg_states_[i];
            
            float theta = leg.theta[0];
            float L0 = leg.L0[0];
            float dot_theta = leg.dot_theta;
            float ddot_theta = leg.ddot_theta;
            float dot_L0 = leg.dot_L0;
            float ddot_L0 = leg.ddot_L0;
            
            // P = F*cos(theta) + Tp*sin(theta)/L0
            float P = F[i] * std::cos(theta) + Tp[i] * std::sin(theta) / L0;
            
            // ddot_zw = ddot_zb - ddot_L0*cos(theta) + 2*dot_L0*dot_theta*sin(theta) + L0*ddot_theta*sin(theta) + L0*(d_theta)^2*cos(theta)
            float ddot_zw = ddot_zb 
                        - ddot_L0 * std::cos(theta)
                        + 2.0f * dot_L0 * dot_theta * std::sin(theta)
                        + L0 * ddot_theta * std::sin(theta)
                        + L0 * dot_theta * dot_theta * std::cos(theta);
            
            // Fn = mw * ddot_zw + mw * g + P
            float Fn = wheel_mass * ddot_zw * 0.0f + wheel_mass * 9.8f + P;
            
            // 移动平均滤波
            Fn_buffer_[i][9] = Fn_buffer_[i][8];
            Fn_buffer_[i][8] = Fn_buffer_[i][7];
            Fn_buffer_[i][7] = Fn_buffer_[i][6];
            Fn_buffer_[i][6] = Fn_buffer_[i][5];
            Fn_buffer_[i][5] = Fn_buffer_[i][4];
            Fn_buffer_[i][4] = Fn_buffer_[i][3];
            Fn_buffer_[i][3] = Fn_buffer_[i][2];
            Fn_buffer_[i][2] = Fn_buffer_[i][1];
            Fn_buffer_[i][1] = Fn_buffer_[i][0];
            Fn_buffer_[i][0] = Fn;
            
            if (data_count_[i] < 10) {
                data_count_[i]++;
            }
            
            float Fn_avg = 0.0f;
            for (int j = 0; j < data_count_[i]; ++j) {
                Fn_avg += Fn_buffer_[i][j];
            }
            Fn_avg /= data_count_[i];
            
            force_estimate_.ddot_zw[i] = ddot_zw;
            force_estimate_.Fn[i] = Fn_avg;
        }
    }
    
    /**
    * @brief 离地检测
    * @return true: 离地, false: 接地
    */
    bool detectGround() {
        float left_Fn = force_estimate_.Fn[0];
        float right_Fn = force_estimate_.Fn[1];
        
        const auto& thresholds = config_.ground_detection;
        
        // 正常支撑力区间
        if ((left_Fn > thresholds.normal_force_min && 
            right_Fn > thresholds.normal_force_min) &&
            (left_Fn < thresholds.normal_force_max && 
            right_Fn < thresholds.normal_force_max)) {
            
            if (force_estimate_.ground_signal_) {
                ground_time_++;
                if (ground_time_ > thresholds.stable_count_threshold) {
                    force_estimate_.ground_signal_ = false;
                    ground_time_ = 0;
                }
            }
        }
        // 离地区间
        else if (left_Fn < thresholds.airborne_threshold && right_Fn < thresholds.airborne_threshold) {
            force_estimate_.ground_signal_ = true;
            ground_time_ = 0;
        }
        // 过载区间
        else if (left_Fn > thresholds.overload_threshold && right_Fn > thresholds.overload_threshold) {
            force_estimate_.ground_signal_ = true;
            ground_time_ = 0;
        }
        else {
            ground_time_ = 0;
        }
        
        return force_estimate_.ground_signal_;
    }
    
    /**
    * @brief 估计速度和位置
    * @param w_ecd 轮子编码器角速度 [left, right]
    * @param b_acc 机体x轴加速度
    * @param b_vel 机体俯仰角速度
    */
    void estimateVelocity(const std::array<float, 2>& w_ecd,
                        float b_acc,
                        float b_vel,
                        float wheel_radius) {
        
        velocity_estimate_.w_ecd = w_ecd;
        velocity_estimate_.b_acc = b_acc;
        velocity_estimate_.b_vel = b_vel;
        
        const float dt = config_.dt;
        
        // 计算轮子速度
        for (int i = 0; i < 2; ++i) {
            float L0 = leg_states_[i].L0[0];
            float theta = leg_states_[i].theta[0];
            float dot_L0 = leg_states_[i].dot_L0;
            float dot_theta = leg_states_[i].dot_theta;
            float dot_alpha = leg_states_[i].dot_alpha;
            
            float w = w_ecd[i] + b_vel + dot_alpha;
            velocity_estimate_.dot_xw[i] = w * wheel_radius
                + L0 * dot_theta * std::cos(theta) 
                + dot_L0 * std::sin(theta);
        }
        
        // 平均速度
        velocity_estimate_.aver_vel = 
            (velocity_estimate_.dot_xw[0] + velocity_estimate_.dot_xw[1]) / 2.0f;
        
        // 卡尔曼滤波
        kalman_filter_.measured_vector << 
            velocity_estimate_.aver_vel, 
            velocity_estimate_.b_acc;
        
        kalman_filter_.update();
        
        // 更新估计值
        // velocity_estimate_.x_filter += velocity_estimate_.v_filter * dt;
        // velocity_estimate_.v_filter = kalman_filter_.x(0);
        
        velocity_estimate_.x_filter += velocity_estimate_.v_filter * dt;
        velocity_estimate_.v_filter = (w_ecd[0] + w_ecd[1]) * wheel_radius / 2;
    }
    
    /**
    * @brief 清零所有状态
    */
    void clear() {
        leg_states_[0] = LegState();
        leg_states_[1] = LegState();
        force_estimate_ = ForceEstimate();
        velocity_estimate_ = VelocityEstimate();
        
        for (auto& buffer : Fn_buffer_) {
            buffer.fill(0.0f);
        }
        data_count_.fill(0);
        
        ground_time_ = 0;
        
        initializeKalmanFilter();
    }

    void corverVelocityEstimate(float position, float velcocity){
        velocity_estimate_.x_filter = position;
        velocity_estimate_.v_filter = velcocity;
    }
    
    // ==================== 访问器 ====================

    const std::array<LegState, 2>& getLegStates() const { 
        return leg_states_; 
    }
    
    const LegState& getLegState(int index) const {
        return leg_states_[index];
    }
    
    const ForceEstimate& getForceEstimate() const { 
        return force_estimate_; 
    }
    
    const VelocityEstimate& getVelocityEstimate() const { 
        return velocity_estimate_; 
    }

    const KalmanFilter& getKalmanFilter() const { 
        return kalman_filter_; 
    }
    
    bool isGrounded() const { 
        return !force_estimate_.ground_signal_; 
    }

    bool isSliped(size_t index) const {
        return !velocity_estimate_.slip_signal_[index];
    }

private:
    /**
    * @brief 初始化卡尔曼滤波器
    */
    void initializeKalmanFilter() {
        const float dt = config_.dt;
        
        // 状态转移矩阵 F
        kalman_filter_.F << 1.0f, dt,
                        0.0f, 1.0f;
        
        // 初始协方差 P
        kalman_filter_.P << 1.0f, 0.0f,
                        0.0f, 1.0f;
        
        // 过程噪声 Q
        kalman_filter_.Q << config_.kalman_filter_param.Q[0], 0.0f,
                        0.0f, config_.kalman_filter_param.Q[1];
        
        // 测量噪声 R
        kalman_filter_.R_diag = {
            config_.kalman_filter_param.R[0], 
            config_.kalman_filter_param.R[1]
        };
        
        kalman_filter_.measurement_map = {1, 2};
        kalman_filter_.measurement_degree = {1, 1};
    }
    
    Config config_;
    
    std::array<LegState, 2> leg_states_;
    ForceEstimate force_estimate_;
    VelocityEstimate velocity_estimate_;
    
    // 卡尔曼滤波器
    KalmanFilter kalman_filter_;
    
    // 支撑力滤波缓冲
    std::array<std::array<float, 10>, 2> Fn_buffer_{};
    std::array<int, 2> data_count_{0};
    
    // 离地检测
    int ground_time_ = 0;
};

} // namespace controller

#endif // STATE_ESTIMATOR_HPP