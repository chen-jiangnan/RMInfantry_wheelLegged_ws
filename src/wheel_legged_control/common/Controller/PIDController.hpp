/**
 * @file PIDController.hpp
 * @brief 现代C++ PID控制器
 */

 #ifndef PID_CONTROLLER_HPP
 #define PID_CONTROLLER_HPP
 
 #include <array>
 #include <algorithm>
 #include <cstring>
 
namespace controller {

/**
* @brief PID控制器类
*/
class PIDController {
public:
    /**
    * @brief PID模式
    */
    enum class Mode {
        POSITION,  // 位置式PID
        DELTA      // 增量式PID
    };
    
    /**
    * @brief PID配置
    */
    struct Config {
        Mode mode = Mode::POSITION;
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        float max_out = 0.0f;
        float max_iout = 0.0f;
        
        // 构造函数
        Config() = default;
        Config(Mode m, float p, float i, float d, float max_o, float max_io)
            : mode(m), kp(p), ki(i), kd(d), max_out(max_o), max_iout(max_io) {}
    };
    
    /**
    * @brief 构造函数
    */
    PIDController() { clear(); }
    
    /**
    * @brief 带配置的构造函数
    */
    explicit PIDController(const Config& config) 
        : config_(config) 
    { 
        clear(); 
    }
    
    /**
    * @brief 设置配置
    */
    void setConfig(const Config& config) {
        config_ = config;
        clear();
    }
    
    /**
    * @brief 获取配置
    */
    const Config& getConfig() const { return config_; }
    
    /**
    * @brief 计算PID输出
    * @param setpoint 设定值
    * @param measurement 测量值
    * @return PID输出
    */
    float calculate(float setpoint, float measurement) {
        // 更新误差历史
        error_[2] = error_[1];
        error_[1] = error_[0];
        error_[0] = setpoint - measurement;
        
        set_ = setpoint;
        fdb_ = measurement;
        
        if (config_.mode == Mode::POSITION) {
            return calculatePosition();
        } else {
            return calculateDelta();
        }
    }
    
    /**
    * @brief 清零
    */
    void clear() {
        error_.fill(0.0f);
        d_buf_.fill(0.0f);
        out_ = 0.0f;
        p_out_ = 0.0f;
        i_out_ = 0.0f;
        d_out_ = 0.0f;
        set_ = 0.0f;
        fdb_ = 0.0f;
    }
    
    /**
    * @brief 获取
    */
    float getSet() const { return set_; }
    float getFdb() const { return fdb_; }
    float getOutput() const { return out_; }
    float getPOut() const { return p_out_; }
    float getIOut() const { return i_out_; }
    float getDOut() const { return d_out_; }
    float getError() const { return error_[0]; }
    const std::array<float, 3>& getErrors() const { return error_; }

private:
    /**
    * @brief 位置式PID计算
    */
    float calculatePosition() {
        // P项
        p_out_ = config_.kp * error_[0];
        
        // I项
        i_out_ += config_.ki * error_[0];
        i_out_ = limit(i_out_, config_.max_iout);
        
        // D项
        d_buf_[2] = d_buf_[1];
        d_buf_[1] = d_buf_[0];
        d_buf_[0] = error_[0] - error_[1];
        d_out_ = config_.kd * d_buf_[0];
        
        // 总输出
        out_ = p_out_ + i_out_ + d_out_;
        out_ = limit(out_, config_.max_out);
        
        return out_;
    }
    
    /**
    * @brief 增量式PID计算
    */
    float calculateDelta() {
        // P项
        p_out_ = config_.kp * (error_[0] - error_[1]);
        
        // I项
        i_out_ = config_.ki * error_[0];
        
        // D项
        d_buf_[2] = d_buf_[1];
        d_buf_[1] = d_buf_[0];
        d_buf_[0] = error_[0] - 2.0f * error_[1] + error_[2];
        d_out_ = config_.kd * d_buf_[0];
        
        // 增量
        out_ += p_out_ + i_out_ + d_out_;
        out_ = limit(out_, config_.max_out);
        
        return out_;
    }
    
    /**
    * @brief 限幅
    */
    static float limit(float value, float max_abs) {
        if (max_abs <= 0.0f) return value;
        return std::clamp(value, -max_abs, max_abs);
    }
    
    Config config_;
    
    std::array<float, 3> error_{0.0f};
    std::array<float, 3> d_buf_{0.0f};
    
    float out_ = 0.0f;
    float p_out_ = 0.0f;
    float i_out_ = 0.0f;
    float d_out_ = 0.0f;
    
    float set_ = 0.0f;
    float fdb_ = 0.0f;
};

} // namespace controller

 #endif // PID_CONTROLLER_HPP