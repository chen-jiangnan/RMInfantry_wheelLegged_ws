/**
 * @file joint_state_interface.hpp
 * @brief JointState消息的C++封装接口
 */

#ifndef JOINT_STATE_INTERFACE_HPP
#define JOINT_STATE_INTERFACE_HPP

#include <array>
#include <vector>
#include <string>
#include <wheel_legged_msgs/msg/detail/joint_state__struct.hpp>
#include <wheel_legged_msgs/msg/detail/joint_states__struct.hpp>
#include "wheel_legged_msgs/msg/joint_state.hpp"
#include "wheel_legged_msgs/msg/joint_states.hpp"

namespace wheel_legged_interfaces {
/**
* @brief JointState接口类
* 
*/
class JointStateInterface {
public:
    using msg_JointState = wheel_legged_msgs::msg::JointState;
    using msg_JointStates = wheel_legged_msgs::msg::JointStates;
    // 控制模式
    enum Mode : uint8_t {
        MODE_IDLE = msg_JointState::MODE_IDLE,
        MODE_POSITION = msg_JointState::MODE_POSITION,
        MODE_VELOCITY = msg_JointState::MODE_VELOCITY,
        MODE_EFFORT = msg_JointState::MODE_EFFORT,
        MODE_MIT = msg_JointState::MODE_MIT
    };
    
    /**
    * @brief 构造函数
    * @param num_joints 关节数量
    * @param default_mode 默认控制模式
    */
    explicit JointStateInterface(size_t num_joints = 0, 
                                Mode default_mode = MODE_IDLE);
    
    // ==================== 设置方法 ====================
    
    void resize(size_t num_joints);
    
    void setNames(const std::vector<std::string>& names);
    void setName(size_t index, const std::string& name);
    
    /**
    * @brief 设置控制模式
    */
    void setMode(size_t index, Mode mode);  // 单个关节
    void setModes(Mode mode);  // 所有关节相同模式
    void setModes(const std::vector<Mode>& modes);  // 每个关节不同
    
    /**
    * @brief 设置单个关节的状态
    */
    void setPosition(size_t index, float value);
    void setVelocity(size_t index, float value);
    void setEffort(size_t index, float value);

    
    /**
    * @brief 批量设置
    */
    void setPositions(const std::vector<float>& values);
    void setVelocities(const std::vector<float>& values);
    void setEfforts(const std::vector<float>& values);
    
    /**
    * @brief 批量设置（数组版本）
    */
    template<size_t N>
    void setPositions(const std::array<float, N>& values) {
        positions_.resize(N);
        for (size_t i = 0; i < N; ++i) {
            positions_[i] = values[i];
        }
    }
    
    template<size_t N>
    void setVelocities(const std::array<float, N>& values) {
        velocities_.resize(N);
        for (size_t i = 0; i < N; ++i) {
            velocities_[i] = values[i];
        }
    }
    
    template<size_t N>
    void setEfforts(const std::array<float, N>& values) {
        efforts_.resize(N);
        for (size_t i = 0; i < N; ++i) {
            efforts_[i] = values[i];
        }
    }
    
    // ==================== 获取方法 ====================
    
    size_t size() const { return modes_.size(); }
    
    Mode getMode(size_t index) const;
    float getPosition(size_t index) const;
    float getVelocity(size_t index) const;
    float getEffort(size_t index) const;
    const std::string& getName(size_t index) const;
    
    const std::vector<float>& getPositions() const { return positions_; }
    const std::vector<float>& getVelocities() const { return velocities_; }
    const std::vector<float>& getEfforts() const { return efforts_; }
    
    /**
    * @brief 设置所有关节为空闲模式
    */
    void idle();

    // ==================== 转换方法 ====================
    msg_JointState toMsg(size_t index) const;
    void fromMsg(size_t index, const msg_JointState& msg);
    msg_JointStates toMsgs() const;
    void fromMsgs(const msg_JointStates& msgs);
    void print() const;

private:
    std::vector<std::string> names_;
    std::vector<Mode> modes_;
    std::vector<float> positions_;
    std::vector<float> velocities_;
    std::vector<float> efforts_;
};

} // namespace wheel_legged_interfaces

#endif // JOINT_STATE_INTERFACE_HPP 