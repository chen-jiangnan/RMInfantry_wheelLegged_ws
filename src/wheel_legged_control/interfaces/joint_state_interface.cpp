/**
 * @file joint_state_interface.cpp
 * @brief JointState消息的C++封装接口实现
 */

#include "joint_state_interface.hpp"
#include <cstddef>
#include <iostream>
#include <iomanip>

namespace wheel_legged_interfaces {

// ==================== 构造函数 ====================

JointStateInterface::JointStateInterface(size_t num_joints, Mode default_mode) {
    resize(num_joints);
    setModes(default_mode);
}

// ==================== 设置方法 ====================

void JointStateInterface::resize(size_t num_joints) {
    names_.resize(num_joints);
    modes_.resize(num_joints, MODE_IDLE);
    positions_.resize(num_joints, 0.0f);
    velocities_.resize(num_joints, 0.0f);
    efforts_.resize(num_joints, 0.0f);
}

void JointStateInterface::setNames(const std::vector<std::string>& names) {
    names_ = names;
}

void JointStateInterface::setName(size_t index, const std::string& name) {
    if (index < names_.size()) {
        names_[index] = name;
    }
}

void JointStateInterface::setMode(size_t index, Mode mode) {
    if (index < modes_.size()) {
        modes_[index] = mode;
    }
}

void JointStateInterface::setModes(Mode mode) {
    for (auto& m : modes_) {
        m = mode;
    }
}

void JointStateInterface::setModes(const std::vector<Mode>& modes) {
    modes_ = modes;
}

void JointStateInterface::setPosition(size_t index, float value) {
    if (index < positions_.size()) {
        positions_[index] = value;
    }
}

void JointStateInterface::setVelocity(size_t index, float value) {
    if (index < velocities_.size()) {
        velocities_[index] = value;
    }
}

void JointStateInterface::setEffort(size_t index, float value) {
    if (index < efforts_.size()) {
        efforts_[index] = value;
    }
}



void JointStateInterface::setPositions(const std::vector<float>& values) {
    positions_ = values;
}

void JointStateInterface::setVelocities(const std::vector<float>& values) {
    velocities_ = values;
}

void JointStateInterface::setEfforts(const std::vector<float>& values) {
    efforts_ = values;
}



// ==================== 获取方法 ====================

JointStateInterface::Mode JointStateInterface::getMode(size_t index) const {
    return index < modes_.size() ? modes_[index] : MODE_IDLE;
}

float JointStateInterface::getPosition(size_t index) const {
    return index < positions_.size() ? positions_[index] : 0.0f;
}

float JointStateInterface::getVelocity(size_t index) const {
    return index < velocities_.size() ? velocities_[index] : 0.0f;
}

float JointStateInterface::getEffort(size_t index) const {
    return index < efforts_.size() ? efforts_[index] : 0.0f;
}



const std::string& JointStateInterface::getName(size_t index) const {
    static const std::string empty;
    return index < names_.size() ? names_[index] : empty;
}

// ==================== 便捷方法 ====================

void JointStateInterface::idle() {
    setModes(MODE_IDLE);
}

// ==================== 转换方法 ====================

JointStateInterface::msg_JointState JointStateInterface::toMsg(size_t index) const {
    msg_JointState msg;

    msg.name = getName(index);
    msg.mode = getMode(index);
    msg.q = getPosition(index);
    msg.dq = getVelocity(index);
    msg.tau = getEffort(index);
    return msg;
}

void JointStateInterface::fromMsg(size_t index, const msg_JointState& msg) {
    setName(index, msg.name);    
    setMode(index, static_cast<Mode>(msg.mode));
    setPosition(index, msg.q);    
    setVelocity(index, msg.dq);
    setEffort(index, msg.tau);
}

JointStateInterface::msg_JointStates JointStateInterface::toMsgs() const{
    msg_JointStates msgs;
    msgs.joint_states.reserve(names_.size());

    for(size_t i = 0; i < names_.size(); i++){
        auto msg = toMsg(i);
        msgs.joint_states.push_back(msg);
    }

    return msgs;
};

void JointStateInterface::fromMsgs(const JointStateInterface::msg_JointStates& msgs){
    if(msgs.joint_states.size() != names_.size() ||  msgs.joint_states.empty()){
        std::cout << "msgs and joint size mismatch or msgs is empty" << std::endl;
        return;
    }
    for(size_t i = 0; i < names_.size(); i++){
        fromMsg(i, msgs.joint_states[i]);
    }
}

void JointStateInterface::print() const {
    std::cout << "\n========== JointState ==========\n";
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
        std::cout << "\n";
    }
    std::cout << "==============================\n";
}



} // namespace wheel_legged_interfaces