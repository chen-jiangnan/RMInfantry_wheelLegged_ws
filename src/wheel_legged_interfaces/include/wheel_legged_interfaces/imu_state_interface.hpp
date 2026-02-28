#ifndef IMU_STATE_INTERFACE_HPP
#define IMU_STATE_INTERFACE_HPP

#include <array>
#include <mutex>
#include <iostream>
#include <iomanip>
#include "wheel_legged_msgs/msg/imu_state.hpp"

namespace wheel_legged_interfaces {

class IMUStateInterface {
public:
    using msg_IMUState = wheel_legged_msgs::msg::IMUState;

    explicit IMUStateInterface() = default;

    // ==================== 设置方法 ====================
    void setQuaternion(const std::array<float, 4>& v) {
        std::lock_guard<std::mutex> lock(mutex_); quaternion_ = v;
    }
    void setGyroscope(const std::array<float, 3>& v) {
        std::lock_guard<std::mutex> lock(mutex_); gyroscope_ = v;
    }
    void setAccelerometer(const std::array<float, 3>& v) {
        std::lock_guard<std::mutex> lock(mutex_); accelerometer_ = v;
    }
    void setRPY(const std::array<float, 3>& v) {
        std::lock_guard<std::mutex> lock(mutex_); rpy_ = v;
    }

    // ==================== 获取方法（返回副本）====================
    std::array<float, 4> getQuaternion()    const { std::lock_guard<std::mutex> lock(mutex_); return quaternion_; }
    std::array<float, 3> getGyroscope()     const { std::lock_guard<std::mutex> lock(mutex_); return gyroscope_; }
    std::array<float, 3> getAccelerometer() const { std::lock_guard<std::mutex> lock(mutex_); return accelerometer_; }
    std::array<float, 3> getRPY()           const { std::lock_guard<std::mutex> lock(mutex_); return rpy_; }

    // ==================== 转换方法 ====================
    void fromMsg(const msg_IMUState& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        quaternion_    = msg.quaternion;
        gyroscope_     = msg.gyroscope;
        accelerometer_ = msg.accelerometer;
        rpy_           = msg.rpy;
    }

    msg_IMUState toMsg() const {
        std::lock_guard<std::mutex> lock(mutex_);
        msg_IMUState msg;
        msg.quaternion    = quaternion_;
        msg.gyroscope     = gyroscope_;
        msg.accelerometer = accelerometer_;
        msg.rpy           = rpy_;
        return msg;
    }

    void print() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "\n========== IMUState ==========\n";
        std::cout << "  Quaternion  : [" << quaternion_[0] << ", " << quaternion_[1]
                  << ", " << quaternion_[2] << ", " << quaternion_[3] << "]\n";
        std::cout << "  RPY         : [" << rpy_[0] << ", " << rpy_[1] << ", " << rpy_[2] << "] rad\n";
        std::cout << "  Gyroscope   : [" << gyroscope_[0] << ", " << gyroscope_[1] << ", " << gyroscope_[2] << "] rad/s\n";
        std::cout << "  Accelero    : [" << accelerometer_[0] << ", " << accelerometer_[1] << ", " << accelerometer_[2] << "] m/s²\n";
        std::cout << "==============================\n";
    }

private:
    mutable std::mutex   mutex_;
    std::array<float, 4> quaternion_{};
    std::array<float, 3> gyroscope_{};
    std::array<float, 3> accelerometer_{};
    std::array<float, 3> rpy_{};
};

} // namespace wheel_legged_interfaces
#endif // IMU_STATE_INTERFACE_HPP