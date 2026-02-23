/**
 * @file joint_cmd_interface.cpp
 * @brief JointCmd消息的C++封装接口实现
 */

#include "wheel_legged_interfaces/joint_cmd_interface.hpp"
#include <cstddef>
#include <iostream>
#include <iomanip>

namespace wheel_legged_interfaces {

// ==================== 构造函数 ====================

JointCmdInterface::JointCmdInterface(size_t num_joints, Mode default_mode) {
    resize(num_joints);
    setModes(default_mode);
}

// ==================== 设置方法 ====================

void JointCmdInterface::resize(size_t num_joints) {
    modes_.resize(num_joints, MODE_IDLE);
    positions_.resize(num_joints, 0.0f);
    velocities_.resize(num_joints, 0.0f);
    efforts_.resize(num_joints, 0.0f);
    kps_.resize(num_joints, 0.0f);
    kds_.resize(num_joints, 0.0f);
}

void JointCmdInterface::setMode(size_t index, Mode mode) {
    if (index < modes_.size()) {
        modes_[index] = mode;
    }
}

void JointCmdInterface::setModes(Mode mode) {
    for (auto& m : modes_) {
        m = mode;
    }
}

void JointCmdInterface::setModes(const std::vector<Mode>& modes) {
    modes_ = modes;
}

void JointCmdInterface::setPosition(size_t index, float value) {
    if (index < positions_.size()) {
        positions_[index] = value;
    }
}

void JointCmdInterface::setVelocity(size_t index, float value) {
    if (index < velocities_.size()) {
        velocities_[index] = value;
    }
}

void JointCmdInterface::setEffort(size_t index, float value) {
    if (index < efforts_.size()) {
        efforts_[index] = value;
    }
}

void JointCmdInterface::setKp(size_t index, float value) {
    if (index < kps_.size()) {
        kps_[index] = value;
    }
}

void JointCmdInterface::setKd(size_t index, float value) {
    if (index < kds_.size()) {
        kds_[index] = value;
    }
}

void JointCmdInterface::setPositions(const std::vector<float>& values) {
    positions_ = values;
}

void JointCmdInterface::setVelocities(const std::vector<float>& values) {
    velocities_ = values;
}

void JointCmdInterface::setEfforts(const std::vector<float>& values) {
    efforts_ = values;
}

void JointCmdInterface::setKps(const std::vector<float>& values) {
    kps_ = values;
}

void JointCmdInterface::setKds(const std::vector<float>& values) {
    kds_ = values;
}

// ==================== 获取方法 ====================

JointCmdInterface::Mode JointCmdInterface::getMode(size_t index) const {
    return index < modes_.size() ? modes_[index] : MODE_IDLE;
}

float JointCmdInterface::getPosition(size_t index) const {
    return index < positions_.size() ? positions_[index] : 0.0f;
}

float JointCmdInterface::getVelocity(size_t index) const {
    return index < velocities_.size() ? velocities_[index] : 0.0f;
}

float JointCmdInterface::getEffort(size_t index) const {
    return index < efforts_.size() ? efforts_[index] : 0.0f;
}

float JointCmdInterface::getKp(size_t index) const {
    return index < kps_.size() ? kps_[index] : 0.0f;
}

float JointCmdInterface::getKd(size_t index) const {
    return index < kds_.size() ? kds_[index] : 0.0f;
}


// ==================== 便捷方法 ====================

void JointCmdInterface::idle() {
    setModes(MODE_IDLE);
}

// ==================== 转换方法 ====================

JointCmdInterface::msg_JointCmd JointCmdInterface::toMsg(size_t index) const {
    msg_JointCmd msg;

    msg.mode = getMode(index);
    msg.q = getPosition(index);
    msg.dq = getVelocity(index);
    msg.tau = getEffort(index);
    msg.kp = getKp(index);
    msg.kd = getKd(index);

    return msg;
}

void JointCmdInterface::fromMsg(size_t index, const msg_JointCmd& msg) {   
    setMode(index, static_cast<Mode>(msg.mode));
    setPosition(index, msg.q);    
    setVelocity(index, msg.dq);
    setEffort(index, msg.tau);
    setKp(index, msg.kp);
    setKd(index, msg.kd);
}

JointCmdInterface::msg_JointCmds JointCmdInterface::toMsgs() const{
    msg_JointCmds msgs;
    msgs.joint_cmds.reserve(modes_.size());

    for(size_t i = 0; i < modes_.size(); i++){
        auto msg = toMsg(i);
        msgs.joint_cmds.push_back(msg);
    }

    return msgs;
};

void JointCmdInterface::fromMsgs(const JointCmdInterface::msg_JointCmds& msgs){
    if(msgs.joint_cmds.size() != modes_.size() ||  msgs.joint_cmds.empty()){
        std::cout << "msgs and joint size mismatch or msgs is empty" << std::endl;
        return;
    }
    for(size_t i = 0; i < modes_.size(); i++){
        fromMsg(i, msgs.joint_cmds[i]);
    }
}

void JointCmdInterface::print() const {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "\n========== JointCmd (" << modes_.size() << " joints) ==========\n";
    std::cout << std::setw(4)  << "idx"
              << std::setw(10) << "mode"
              << std::setw(10) << "pos"
              << std::setw(10) << "vel"
              << std::setw(10) << "eff"
              << std::setw(8)  << "kp"
              << std::setw(8)  << "kd" << "\n";
    std::cout << std::string(60, '-') << "\n";

    auto modeStr = [](Mode m) -> std::string {
        switch (m) {
            case MODE_IDLE:     return "IDLE";
            case MODE_POSITION: return "POS";
            case MODE_VELOCITY: return "VEL";
            case MODE_EFFORT:   return "EFF";
            case MODE_MIT:      return "MIT";
            default:            return "?";
        }
    };

    for (size_t i = 0; i < modes_.size(); ++i) {
        std::cout << std::setw(4)  << i
                  << std::setw(10) << modeStr(modes_[i])
                  << std::setw(10) << (i < positions_.size()  ? positions_[i]  : 0.f)
                  << std::setw(10) << (i < velocities_.size() ? velocities_[i] : 0.f)
                  << std::setw(10) << (i < efforts_.size()    ? efforts_[i]    : 0.f)
                  << std::setw(8)  << (i < kps_.size()        ? kps_[i]        : 0.f)
                  << std::setw(8)  << (i < kds_.size()        ? kds_[i]        : 0.f)
                  << "\n";
    }
    std::cout << "=============================================\n";
}
} // namespace wheel_legged_interfaces