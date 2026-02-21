/**
 * @file joint_state_interface.hpp
 * @brief JointState消息的C++封装接口
 */

#ifndef IMU_STATE_INTERFACE_HPP
#define IMU_STATE_INTERFACE_HPP

#include <array>
#include <vector>
#include <string>
#include "wheel_legged_msgs/msg/imu_state.hpp"

namespace wheel_legged_interfaces {
/**
* @brief JoinState接口类
* 
*/
class IMUStateInterface {
public:
    using msg_IMUState = wheel_legged_msgs::msg::IMUState;

    /**
    * @brief 构造函数
    */
    explicit IMUStateInterface() = default;
    
    // ==================== 设置方法 ====================
    void setQuaternion(const std::array<float, 4>& values) {
        quaternion_ = values;
    }
    
    void setQyroscope(const std::array<float, 3>& values) {
        gyroscope_ = values;
    }

    void setAccelerometer(const std::array<float, 3>& values) {
        accelerometer_ = values;
    }

    void setRPY(const std::array<float, 3>& values) {
        rpy_ = values;
    }

    // ==================== 获取方法 ====================

    const std::array<float, 4>& getQuaternion() const { return quaternion_; }
    const std::array<float, 3>& getGyroscope() const { return gyroscope_; }
    const std::array<float, 3>& getAccelerometer() const { return accelerometer_; }
    const std::array<float, 3>& getRPY() const { return rpy_; }

    // ==================== 转换方法 ====================
    msg_IMUState toMsg() const { 
        msg_IMUState msg;

        msg.quaternion = getQuaternion();
        msg.gyroscope = getGyroscope();
        msg.accelerometer = getAccelerometer();
        msg.rpy = getRPY();

        return msg;
    };

    void fromMsg(const msg_IMUState& msg){
        setQuaternion(msg.quaternion);
        setQyroscope(msg.gyroscope);
        setAccelerometer(msg.accelerometer);
        setRPY(msg.rpy);
    }

private:
    std::array<float, 4> quaternion_;
    std::array<float, 3> gyroscope_;
    std::array<float, 3> accelerometer_;
    std::array<float, 3> rpy_;
};

} // namespace wheel_legged_interfaces

#endif // IMU_STATE_INTERFACE_HPP 