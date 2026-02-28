#ifndef CHASSIS_CTRL_INTERFACE_HPP
#define CHASSIS_CTRL_INTERFACE_HPP

#include <array>
#include <mutex>
#include <string>
#include <iostream>
#include <iomanip>
#include "joint_fsm.hpp"
#include "wheel_legged_msgs/msg/chassis_ctrl.hpp"

namespace wheel_legged_interfaces {

class ChassisCtrlInterface {
public:
    using msg_ChassisCtrl = wheel_legged_msgs::msg::ChassisCtrl;

    explicit ChassisCtrlInterface() = default;

    // ==================== 设置方法 ====================
    void setMode(JFSMode mode)              { std::lock_guard<std::mutex> lock(mutex_); joint_fsm_mode_ = mode; }
    void setPosition(float v)               { std::lock_guard<std::mutex> lock(mutex_); position_ = v; }
    void setVelocity(float v)               { std::lock_guard<std::mutex> lock(mutex_); velocity_ = v; }
    void setYawSpeed(float v)               { std::lock_guard<std::mutex> lock(mutex_); yaw_speed_ = v; }
    void setRollEuler(float v)              { std::lock_guard<std::mutex> lock(mutex_); roll_euler_ = v; }
    void setLegLength(size_t i, float v)    { std::lock_guard<std::mutex> lock(mutex_); leg_lengths_[i] = v; }
    void setFootJumpForce(size_t i, float v){ std::lock_guard<std::mutex> lock(mutex_); foot_jump_forces_[i] = v; }
    void setLegLengths(std::array<float,2> v)    { std::lock_guard<std::mutex> lock(mutex_); leg_lengths_ = v; }
    void setFootJumpForces(std::array<float,2> v){ std::lock_guard<std::mutex> lock(mutex_); foot_jump_forces_ = v; }

    // ==================== 获取方法（返回副本）====================
    JFSMode              getMode()                   const { std::lock_guard<std::mutex> lock(mutex_); return joint_fsm_mode_; }
    float                getPosition()               const { std::lock_guard<std::mutex> lock(mutex_); return position_; }
    float                getVelocity()               const { std::lock_guard<std::mutex> lock(mutex_); return velocity_; }
    float                getYawSpeed()               const { std::lock_guard<std::mutex> lock(mutex_); return yaw_speed_; }
    float                getRollEuler()              const { std::lock_guard<std::mutex> lock(mutex_); return roll_euler_; }
    float                getLegLength(size_t i)      const { std::lock_guard<std::mutex> lock(mutex_); return leg_lengths_[i]; }
    float                getFootJumpForce(size_t i)  const { std::lock_guard<std::mutex> lock(mutex_); return foot_jump_forces_[i]; }
    std::array<float, 2> getLegLengths()             const { std::lock_guard<std::mutex> lock(mutex_); return leg_lengths_; }
    std::array<float, 2> getFootJumpForces()         const { std::lock_guard<std::mutex> lock(mutex_); return foot_jump_forces_; }

    // ==================== 转换方法 ====================
    void fromMsg(const msg_ChassisCtrl& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        joint_fsm_mode_    = static_cast<JFSMode>(msg.joint_fsm_mode);
        position_          = msg.position;
        velocity_          = msg.velocity;
        yaw_speed_         = msg.yaw_speed;
        roll_euler_        = msg.roll_euler;
        leg_lengths_[0]    = msg.leg_lengths[0];
        leg_lengths_[1]    = msg.leg_lengths[1];
        foot_jump_forces_[0] = msg.foot_jump_forces[0];
        foot_jump_forces_[1] = msg.foot_jump_forces[1];
    }

    msg_ChassisCtrl toMsg() const {
        std::lock_guard<std::mutex> lock(mutex_);
        msg_ChassisCtrl msg;
        msg.joint_fsm_mode    = static_cast<uint8_t>(joint_fsm_mode_);
        msg.position          = position_;
        msg.velocity          = velocity_;
        msg.yaw_speed         = yaw_speed_;
        msg.roll_euler        = roll_euler_;
        msg.leg_lengths[0]    = leg_lengths_[0];
        msg.leg_lengths[1]    = leg_lengths_[1];
        msg.foot_jump_forces[0] = foot_jump_forces_[0];
        msg.foot_jump_forces[1] = foot_jump_forces_[1];
        return msg;
    }

    void print() const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto modeStr = [](JFSMode m) -> std::string {
            switch (m) {
                case JFSMode::ZEROTAU: return "ZEROTAU";
                case JFSMode::CALI:    return "CALI";
                case JFSMode::DAMPING: return "DAMPING";
                case JFSMode::RESET:   return "RESET";
                case JFSMode::READY:   return "READY";
                default:               return "UNKNOWN";
            }
        };
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "\n========== ChassisCtrl ==========\n";
        std::cout << "  FSM Mode   : " << modeStr(joint_fsm_mode_) << "\n";
        std::cout << "  Position   : " << position_   << " m\n";
        std::cout << "  Velocity   : " << velocity_   << " m/s\n";
        std::cout << "  Yaw Speed  : " << yaw_speed_  << " rad/s\n";
        std::cout << "  Roll Euler : " << roll_euler_ << " rad\n";
        std::cout << "  Leg Lengths: [" << leg_lengths_[0] << ", " << leg_lengths_[1] << "] m\n";
        std::cout << "  Jump Forces: [" << foot_jump_forces_[0] << ", " << foot_jump_forces_[1] << "] N\n";
        std::cout << "=================================\n";
    }

private:
    mutable std::mutex   mutex_;
    JFSMode              joint_fsm_mode_{};
    float                position_{};
    float                velocity_{};
    float                yaw_speed_{};
    float                roll_euler_{};
    std::array<float, 2> leg_lengths_{};
    std::array<float, 2> foot_jump_forces_{};
};

} // namespace wheel_legged_interfaces
#endif // CHASSIS_CTRL_INTERFACE_HPP