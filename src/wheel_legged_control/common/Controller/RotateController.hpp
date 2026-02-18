/**
 * @file RotateController.hpp
 * @brief 现代C++ 旋转控制器
 */

 #ifndef ROTATE_CONTROLLER_HPP
 #define ROTATE_CONTROLLER_HPP
 
 #include "PIDController.hpp"
 
 namespace controller {
 
/**
* @brief 旋转控制器类
* 
* 控制机体绕Z轴旋转（Yaw角速度控制）
*/
class RotateController {
public:
    /**
    * @brief 旋转控制器配置
    */
    struct Config {
        PIDController::Config pid;
        float max_out = 3.0f;
        float max_iout = 3.0f;
        
        Config() {
            // 默认PID参数
            pid = PIDController::Config(
                PIDController::Mode::POSITION,
                2.0f, 0.0f, 0.5f,
                max_out, max_iout
            );
        }
    };
    
    /**
    * @brief 构造函数
    */
    RotateController() {
        setConfig(Config());
    }
    
    /**
    * @brief 带配置的构造函数
    */
    explicit RotateController(const Config& config) {
        setConfig(config);
    }
    
    /**
    * @brief 设置配置
    */
    void setConfig(const Config& config) {
        config_ = config;
        pid_.setConfig(config_.pid);
    }
    
    /**
    * @brief 获取配置
    */
    const Config& getConfig() const { return config_; }
    
    /**
    * @brief 计算控制输出
    * @param target_yaw_rate 目标角速度 (rad/s)
    * @param current_yaw_rate 当前角速度 (rad/s)
    * @return 控制输出（力矩分配系数）
    */
    float calculate(float target_yaw_rate, float current_yaw_rate) {
        return pid_.calculate(target_yaw_rate, current_yaw_rate);
    }
    
    /**
    * @brief 清零
    */
    void clear() {
        pid_.clear();
    }
    
    /**
    * @brief 获取PID控制器（用于调试）
    */
    const PIDController& getPID() const { return pid_; }

private:
    Config config_;
    PIDController pid_;
};

} // namespace controller
 
 #endif // ROTATE_CONTROLLER_HPP