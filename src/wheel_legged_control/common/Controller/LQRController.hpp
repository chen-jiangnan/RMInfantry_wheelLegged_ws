/**
 * @file LQRController.hpp
 * @brief 现代C++ LQR控制器
 */

#ifndef LQR_CONTROLLER_HPP
#define LQR_CONTROLLER_HPP

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include "iostream"
#include <iomanip>
namespace controller {

/**
* @brief LQR控制器类
* 
* 控制律: u = -K * (x - x_d)
* 
* @tparam StateSize 状态维度
* @tparam ControlSize 控制维度
* @tparam PolyOrder 多项式系数
*/
template<int StateSize = 6, int ControlSize = 2, int PolyOrder = 3>
class LQRController {
public:
    using StateVector = Eigen::Matrix<float, StateSize, 1>;
    using ControlVector = Eigen::Matrix<float, ControlSize, 1>;
    using GainMatrix = Eigen::Matrix<float, ControlSize, StateSize>;
    
    /**
    * @brief LQR配置
    */
    struct Config {
        // 增益矩阵 [left, right]
        std::array<GainMatrix ,2> K;       

        // 腿长拟合参数
        bool leg_fitting_enabled = false;
        float min_leg_length = 0.0f;
        float max_leg_length = 0.0f;
        
        // 拟合系数：K[i][j] = p1*L^2 + p2*L + p3
        // [ControlSize * StateSize][3]
        std::array<std::array<float, PolyOrder>, ControlSize * StateSize> fit_coeffs;
        
        Config() {
            for(size_t index = 0; index < 2; index++){
                K[index].setZero();
                for (auto& coeff : fit_coeffs) {
                    coeff.fill(0.0f);
                }
            }
        }
        
        // 从数组设置K矩阵
        void setKMatrix(size_t index, const float* data) {
            if (index > 1){return;}
            for (int i = 0; i < ControlSize; ++i) {
                for (int j = 0; j < StateSize; ++j) {
                    K[index](i, j) = data[i * StateSize + j];
                }
            }
        }
        
        // 从二维数组设置K矩阵
        void setKMatrix(size_t index, const float data[ControlSize][StateSize]) {
            if (index > 1){return;}
            for (int i = 0; i < ControlSize; ++i) {
                for (int j = 0; j < StateSize; ++j) {
                    K[index](i, j) = data[i][j];
                }
            }
        }
    };
    
    /**
    * @brief 构造函数
    */
    LQRController() = default;
    
    /**
    * @brief 带配置的构造函数
    */
    explicit LQRController(const Config& config) 
        : config_(config) {}
    
    /**
    * @brief 设置配置
    */
    void setConfig(const Config& config) {
        config_ = config;
    }
    
    /**
    * @brief 获取配置
    */
    const Config& getConfig() const { return config_; }
    
    /**
    * @brief 计算控制输出
    * @param target_state 目标状态
    * @param current_state 当前状态
    * @return 控制输出
    */
    std::array<ControlVector, 2> calculate(
        const StateVector& target_state,
        const std::array<StateVector, 2>& current_state
    ){
        std::array<ControlVector, 2> controls;
        for(size_t index = 0; index < 2; index++){
            X_[index] = current_state[index];
            Xd_[index] = target_state;
            Err_[index] = Xd_[index] - X_[index];
            U_[index] = config_.K[index] * Err_[index];
            controls[index] = U_[index];
        }
        return controls;
    }

    /**
    * @brief 更新腿长（用于拟合）
    * @param leg_length 当前腿长
    */
    void updateKFormLegLength(size_t index, float leg_length) {
        if (!config_.leg_fitting_enabled || index > 1) {
            return;
        }
        
        // 限制范围
        leg_length = std::clamp(leg_length, 
                            config_.min_leg_length, 
                            config_.max_leg_length);
        
        // 更新K矩阵：K[i][j] = p1*L^2 + p2*L + p3
        for (int i = 0; i < ControlSize; ++i) {
            for (int j = 0; j < StateSize; ++j) {
                int idx = i * StateSize + j;
                float p1 = config_.fit_coeffs[idx][0];
                float p2 = config_.fit_coeffs[idx][1];
                float p3 = config_.fit_coeffs[idx][2];
                
                config_.K[index](i, j) = p1 * leg_length * leg_length + 
                                         p2 * leg_length + 
                                         p3;
            }
        }
        
    }

    void updateKFormLegLength(std::array<float, 2> legs_length) {
        for(size_t index = 0; index < 2; index++){
            // 限制范围
            float leg_length = std::clamp(legs_length[index], 
                                config_.min_leg_length, 
                                config_.max_leg_length);
            
            // 更新K矩阵：K[i][j] = p1*L^2 + p2*L + p3
            for (int i = 0; i < ControlSize; ++i) {
                for (int j = 0; j < StateSize; ++j) {
                    int idx = i * StateSize + j;
                    float p1 = config_.fit_coeffs[idx][0];
                    float p2 = config_.fit_coeffs[idx][1];
                    float p3 = config_.fit_coeffs[idx][2];
                    
                    config_.K[index](i, j) = p1 * leg_length * leg_length + 
                                             p2 * leg_length + 
                                             p3;
                }
            }
        }
    }
    /**
    * @brief 设置当前增益矩阵
    */
    void setGainMatrix(size_t index, size_t rows, size_t cols, float value){
        config_.K[index](rows,cols) = value;}

    /**
    * @brief 获取当前增益矩阵
    */
    const GainMatrix& getGainMatrix(size_t index) const { return config_.K[index]; }
    const StateVector& getXMatrix(size_t index) const { return X_[index]; }
    const StateVector& getXdMatrix(size_t index) const { return Xd_[index]; }
    const StateVector& getErrMatrix(size_t index) const { return Err_[index]; }
    const ControlVector& getUMatrix(size_t index) const { return U_[index]; }

    void printDebugInfo() const {
        std::cout << "\n=========== LQR Debug (Both Sides) ===========\n";
        std::cout << std::fixed << std::setprecision(4);
    
        for (size_t index = 0; index < 2; ++index) {
            std::cout << "\n--- " << (index == 0 ? "Left" : "Right") << " ---\n";
    
            std::cout << "X   : ";
            for (int i = 0; i < StateSize; ++i)
                std::cout << std::setw(9) << X_[index](i);
            std::cout << "\n";
    
            std::cout << "Err : ";
            for (int i = 0; i < StateSize; ++i)
                std::cout << std::setw(9) << Err_[index](i);
            std::cout << "\n";
    
            std::cout << "U   : ";
            for (int i = 0; i < ControlSize; ++i)
                std::cout << std::setw(9) << U_[index](i);
            std::cout << "\n";
        }
    
        std::cout << "\nK Matrices:\n";
    
        for (size_t index = 0; index < 2; ++index) {
            std::cout << (index == 0 ? "Left K:\n" : "Right K:\n");
            for (int r = 0; r < ControlSize; ++r) {
                std::cout << "  ";
                for (int c = 0; c < StateSize; ++c)
                    std::cout << std::setw(9) << config_.K[index](r, c);
                std::cout << "\n";
            }
        }
    
        std::cout << "================================================\n";
    }
        
    

private:
    Config config_;
    std::array<StateVector, 2> X_;      // 状态矩阵
    std::array<StateVector, 2> Xd_;     // 目标矩阵
    std::array<StateVector, 2> Err_;    // 误差矩阵
    std::array<ControlVector, 2> U_;    // 控制矩阵
};
    using LQRController6x2 =  LQRController<6, 2, 3>;
} // namespace controller
 
#endif // LQR_CONTROLLER_HPP