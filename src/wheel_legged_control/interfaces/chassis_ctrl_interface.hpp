/**
 * @file joint_state_interface.hpp
 * @brief JointState消息的C++封装接口
 */

#ifndef WHASSIS_CTRL_INTERFACE_HPP
#define WHASSIS_CTRL_INTERFACE_HPP

#include <array>
#include <cstddef>
#include <vector>
#include <string>
#include "joint_fsm.hpp"
#include "wheel_legged_msgs/msg/chassis_ctrl.hpp"


namespace wheel_legged_interfaces {
/**
* @brief JointState接口类
* 
*/
class ChassisCtrlInterface {
public:

    using msg_ChassisCtrl = wheel_legged_msgs::msg::ChassisCtrl;

    /**
    * @brief 构造函数
    */
    explicit ChassisCtrlInterface() = default;
    
    // ==================== 设置方法 ====================
    /**
    * @brief 设置JFSM模式
    */
    void setMode(JFSMode mode){joint_fsm_mode_ = mode;}  

    /**
    * @brief 设置位移/速度
    */
    void setPosition(float value){position_ = value;}
    void setVelocity(float value){velocity_ = value;}
    void setYawSpeed(float value){yaw_speed_ = value;}

    /**
    * @brief 设置腿
    */ 
    void setRollEuler(float value){roll_euler_ = value;}

    void setLegLength(size_t index, float value){leg_lengths_[index] = value;}
    void setFootJumpForce(size_t index, float value){foot_jump_forces_[index] = value;}

    void setLegLengths(std::array<float, 2> values){
        setLegLength(0, values[0]);
        setLegLength(1, values[1]);
    };
    void setFootJumpForces(std::array<float, 2> values){
        setFootJumpForce(0, values[0]);
        setFootJumpForce(1, values[1]);
    } 

    // ==================== 获取方法 ====================
    JFSMode getMode() const {return joint_fsm_mode_;};
    float getPosition() const {return position_;};
    float getVelocity() const {return velocity_;};
    float getYawSpeed() const {return yaw_speed_;};

    float getLegLength(size_t index) const { return leg_lengths_[index]; }
    float getFootJumpForce(size_t index) const { return foot_jump_forces_[index];}

    const std::array<float, 2>& getLegLengths() const { return leg_lengths_; }
    const std::array<float, 2>& getFootJumpForces() const { return foot_jump_forces_;}
    

    // ==================== 转换方法 ====================
    msg_ChassisCtrl toMsg() const {
        msg_ChassisCtrl msg;
        msg.joint_fsm_mode = static_cast<uint8_t>(joint_fsm_mode_);
        msg.position = position_;
        msg.velocity = velocity_;
        msg.yaw_speed = yaw_speed_;
        msg.roll_euler = roll_euler_;
        msg.leg_lengths[0] = leg_lengths_[0];
        msg.leg_lengths[1] = leg_lengths_[1];
        msg.foot_jump_forces[0] = foot_jump_forces_[0];
        msg.foot_jump_forces[1] = foot_jump_forces_[1];

        return msg;
    };
    void fromMsg(const msg_ChassisCtrl& msg){
        joint_fsm_mode_ = static_cast<JFSMode>(msg.joint_fsm_mode);
        position_ = msg.position;
        velocity_ = msg.velocity;
        yaw_speed_ = msg.yaw_speed;
        roll_euler_ = msg.roll_euler;
        leg_lengths_[0] = msg.leg_lengths[0];
        leg_lengths_[1] = msg.leg_lengths[1];
        foot_jump_forces_[0] = msg.foot_jump_forces[0];
        foot_jump_forces_[1] = msg.foot_jump_forces[1];
    }

    void print() const {
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
        std::cout << "  FSM Mode       : " << modeStr(joint_fsm_mode_) << "\n";
        std::cout << "  Position       : " << std::setw(9) << position_   << " m\n";
        std::cout << "  Velocity       : " << std::setw(9) << velocity_   << " m/s\n";
        std::cout << "  Yaw Speed      : " << std::setw(9) << yaw_speed_  << " rad/s\n";
        std::cout << "  Roll Euler     : " << std::setw(9) << roll_euler_ << " rad\n";
        std::cout << "  Leg Lengths    : ["
                  << std::setw(9) << leg_lengths_[0] << ", "
                  << std::setw(9) << leg_lengths_[1] << "] m\n";
        std::cout << "  Jump Forces    : ["
                  << std::setw(9) << foot_jump_forces_[0] << ", "
                  << std::setw(9) << foot_jump_forces_[1] << "] N\n";
        std::cout << "=================================\n";
    }

private:
    JFSMode joint_fsm_mode_;
    float position_;
    float velocity_;
    float yaw_speed_;
    float roll_euler_;
    std::array<float, 2> leg_lengths_;
    std::array<float, 2> foot_jump_forces_;
};
} // namespace wheel_legged_interfaces

#endif // WHASSIS_CTRL_INTERFACE_HPP 