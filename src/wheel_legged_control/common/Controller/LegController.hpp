/**
* @file LegController.hpp
* @brief 现代C++ 腿控制器
*/

#ifndef LEG_CONTROLLER_HPP
#define LEG_CONTROLLER_HPP

#include "PIDController.hpp"
#include <array>
#include <cmath>

namespace controller {

/**
* @brief 腿控制器类
* 
* 功能:
* 1. 弹簧阻尼控制（腿长控制）
* 2. Roll角补偿
* 3. Phi0补偿
*/
class LegController {
public:
    /**
    * @brief 腿控制器配置
    */
    struct Config {
        // 弹簧阻尼控制
        struct {
            PIDController::Config pid;
            float max_out = 300.0f;
            float max_iout = 300.0f;
        } spring_damping;
        
        // Roll补偿
        struct {
            PIDController::Config pid;
            float max_out = 300.0f;
            float max_iout = 300.0f;
        } roll_compensation;
        
        // Phi0补偿
        struct {
            PIDController::Config pid;
            float max_out = 10.0f;
            float max_iout = 10.0f;
        } phi0_compensation;
        
        Config() {
            // 默认PID参数
            spring_damping.pid = PIDController::Config(
                PIDController::Mode::POSITION, 1000.0f, 0.0f, 200.0f, 
                spring_damping.max_out, spring_damping.max_iout
            );
            
            roll_compensation.pid = PIDController::Config(
                PIDController::Mode::POSITION, 0.0f, 0.0f, 0.0f,
                roll_compensation.max_out, roll_compensation.max_iout
            );
            
            phi0_compensation.pid = PIDController::Config(
                PIDController::Mode::POSITION, 0.0f, 0.0f, 0.0f,
                phi0_compensation.max_out, phi0_compensation.max_iout
            );
        }
    };
    
    /**
    * @brief 控制输出
    */
    struct ControlOutput {
        std::array<float, 2> F;               // 沿腿方向的力
        std::array<float, 2> compensation_tp; // 补偿力矩
    };
    
    /**
    * @brief 构造函数
    */
    LegController() { setConfig(Config()); }
    
    /**
    * @brief 带配置的构造函数
    */
    explicit LegController(const Config& config) {
        setConfig(config);
    }
    
    /**
    * @brief 设置配置
    */
    void setConfig(const Config& config) {
        config_ = config;
        
        // 配置PID控制器
        for (int i = 0; i < 2; ++i) {
            spring_damping_[i].setConfig(config_.spring_damping.pid);
        }
        roll_compensation_.setConfig(config_.roll_compensation.pid);
        phi0_compensation_.setConfig(config_.phi0_compensation.pid);
    }
    
    /**
    * @brief 获取配置
    */
    const Config& getConfig() const { return config_; }
    
    /**
    * @brief 计算控制输出
    * 
    * @param target_L0 目标腿长 [left, right]
    * @param leg_state 腿状态 [left, right]
    * @param target_roll 目标Roll角
    * @param current_roll 当前Roll角
    * @param feedforward_fs 跳跃前馈力[left, right]
    * @return 控制输出
    */
    ControlOutput calculate(
        const std::array<float, 2>& target_L0,
        const std::array<float, 2>& current_L0,
        float target_roll,
        float current_roll,
        const std::array<float, 2>& current_theta,
        const std::array<float, 2>& current_phi0,
        float body_gravity,
        const std::array<float, 2>* feedforward_fs = nullptr
    ) {
        ControlOutput output;
        
        // 1. 弹簧阻尼控制
        for (int i = 0; i < 2; ++i) {
            float spring_force = spring_damping_[i].calculate(
                target_L0[i], 
                current_L0[i]
            );
            
            // 重力前馈
            gravity_compensation_[i] = 
                body_gravity * std::cos(current_theta[i]) / 2.0f;
            
            output.F[i] = spring_force + gravity_compensation_[i];
        }
        
        // 2. Roll补偿
        float roll_compensation = roll_compensation_.calculate(
            target_roll, 
            current_roll
        );
        output.F[0] += roll_compensation;
        output.F[1] -= roll_compensation;
        
        // 3. Phi0补偿
        float diff_phi0 = current_phi0[0] - current_phi0[1];
        float phi0_compensation = phi0_compensation_.calculate(
          0.0f, 
          diff_phi0
        );
        output.compensation_tp[0] =  phi0_compensation;
        output.compensation_tp[1] = -phi0_compensation;
        
        // 4. 跳跃前馈力
        if (feedforward_fs) {
            feedforward_fs_[0] = (*feedforward_fs)[0];
            feedforward_fs_[1] = (*feedforward_fs)[1];
            output.F[0] += feedforward_fs_[0];
            output.F[1] += feedforward_fs_[1];
        }

        tp_ = output.compensation_tp;
        f_ = output.F; 
        return output;
    }
    
    /**
    * @brief 清零所有PID
    */
    void clear() {
        for (auto& pid : spring_damping_) {
            pid.clear();
        }
        roll_compensation_.clear();
        phi0_compensation_.clear();
        gravity_compensation_.fill(0.0f);
        feedforward_fs_.fill(0.0f);
        tp_.fill(0.0f);
        f_.fill(0.0f);
    }
    
    /**
    * @brief 获取PID控制器（用于调试）
    */
    const PIDController& getSpringDampingPID(int index) const {
        return spring_damping_[index];
    }
    
    const PIDController& getRollCompensationPID() const {
        return roll_compensation_;
    }
    
    const PIDController& getPhi0CompensationPID() const {
        return phi0_compensation_;
    }

    const std::array<float, 2>& getGravityCompensation() const {
        return gravity_compensation_;
    }
    const std::array<float, 2>& getFeedforwardForce() const {
        return feedforward_fs_;
    }
    const std::array<float, 2>& getOutPutTp() const {
        return tp_;
    }
    const std::array<float, 2>& getOutPutF() const {
        return f_;
    }



private:
    Config config_;
    
    std::array<PIDController, 2> spring_damping_;   // 左右腿弹簧阻尼
    PIDController roll_compensation_;               // Roll补偿
    PIDController phi0_compensation_;               // Phi0补偿
    std::array<float, 2> gravity_compensation_{};     // 抵消机体自重的力源
    std::array<float, 2> feedforward_fs_{};           // 机体跳跃的额外力源
    std::array<float, 2> tp_{};                       // 补偿力矩
    std::array<float, 2> f_{};                        // 总输出

};

} // namespace controller

#endif // LEG_CONTROLLER_HPP