/**
 * @file joint_cmd_interface.cpp
 * @brief JointCmd消息的C++封装接口实现
 */

#include "joint_cmd_interface.hpp"
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
    names_.resize(num_joints);
    modes_.resize(num_joints, MODE_IDLE);
    positions_.resize(num_joints, 0.0f);
    velocities_.resize(num_joints, 0.0f);
    efforts_.resize(num_joints, 0.0f);
    kps_.resize(num_joints, 0.0f);
    kds_.resize(num_joints, 0.0f);
}

void JointCmdInterface::setNames(const std::vector<std::string>& names) {
    names_ = names;
}

void JointCmdInterface::setName(size_t index, const std::string& name) {
    if (index < names_.size()) {
        names_[index] = name;
    }
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

const std::string& JointCmdInterface::getName(size_t index) const {
    static const std::string empty;
    return index < names_.size() ? names_[index] : empty;
}

// ==================== 便捷方法 ====================

void JointCmdInterface::idle() {
    setModes(MODE_IDLE);
}

// ==================== 转换方法 ====================

JointCmdInterface::msg_JointCmd JointCmdInterface::toMsg(size_t index) const {
    msg_JointCmd msg;

    msg.name = getName(index);
    msg.mode = getMode(index);
    msg.q = getPosition(index);
    msg.dq = getVelocity(index);
    msg.tau = getEffort(index);
    msg.kp = getKp(index);
    msg.kd = getKd(index);

    return msg;
}

void JointCmdInterface::fromMsg(size_t index, const msg_JointCmd& msg) {
    setName(index, msg.name);    
    setMode(index, static_cast<Mode>(msg.mode));
    setPosition(index, msg.q);    
    setVelocity(index, msg.dq);
    setEffort(index, msg.tau);
    setKp(index, msg.kp);
    setKd(index, msg.kd);
}

JointCmdInterface::msg_JointCmds JointCmdInterface::toMsgs() const{
    msg_JointCmds msgs;
    msgs.joint_cmds.reserve(names_.size());

    for(size_t i = 0; i < names_.size(); i++){
        auto msg = toMsg(i);
        msgs.joint_cmds.push_back(msg);
    }

    return msgs;
};

void JointCmdInterface::fromMsgs(const JointCmdInterface::msg_JointCmds& msgs){
    if(msgs.joint_cmds.size() != names_.size() ||  msgs.joint_cmds.empty()){
        std::cout << "msgs and joint size mismatch or msgs is empty" << std::endl;
        return;
    }
    for(size_t i = 0; i < names_.size(); i++){
        fromMsg(i, msgs.joint_cmds[i]);
    }
}

void JointCmdInterface::print() const {
    std::cout << "\n========== JointCmd ==========\n";
    std::cout << std::fixed << std::setprecision(3);
    
    const char* mode_names[] = {
        "IDLE", "POSITION", "VELOCITY", "EFFORT", "MIT"
    };
    
    for (size_t i = 0; i < modes_.size(); ++i) {
        std::cout << "[" << i << "] ";
        if (i < names_.size() && !names_[i].empty()) {
            std::cout << names_[i] << ": ";
        }
        
        std::cout << "mode=" << mode_names[static_cast<int>(modes_[i])];
        
        if (i < positions_.size()) {
            std::cout << " q=" << positions_[i];
        }
        if (i < velocities_.size()) {
            std::cout << " dq=" << velocities_[i];
        }
        if (i < efforts_.size()) {
            std::cout << " tau=" << efforts_[i];
        }
        if (i < kps_.size()) {
            std::cout << " kp=" << kps_[i];
        }
        if (i < kds_.size()) {
            std::cout << " kd=" << kds_[i];
        }
        std::cout << "\n";
    }
    std::cout << "==============================\n";
}

} // namespace wheel_legged_interfaces