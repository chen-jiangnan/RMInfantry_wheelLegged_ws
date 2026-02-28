/**
 * @file joint_state_interface.hpp
 * @brief JointState消息的C++封装接口（线程安全版）
 */

 #ifndef JOINT_STATE_INTERFACE_HPP
 #define JOINT_STATE_INTERFACE_HPP
 
 #include <array>
 #include <vector>
 #include <string>
 #include <mutex>
 #include "wheel_legged_msgs/msg/joint_state.hpp"
 #include "wheel_legged_msgs/msg/joint_states.hpp"
 
 namespace wheel_legged_interfaces {
 
 class JointStateInterface {
 public:
     using msg_JointState  = wheel_legged_msgs::msg::JointState;
     using msg_JointStates = wheel_legged_msgs::msg::JointStates;
 
     enum Mode : uint8_t {
         MODE_IDLE     = msg_JointState::MODE_IDLE,
         MODE_POSITION = msg_JointState::MODE_POSITION,
         MODE_VELOCITY = msg_JointState::MODE_VELOCITY,
         MODE_EFFORT   = msg_JointState::MODE_EFFORT,
         MODE_MIT      = msg_JointState::MODE_MIT
     };
 
     explicit JointStateInterface(size_t num_joints = 0,
                                  Mode default_mode = MODE_IDLE);
 
     // ==================== 设置方法（加锁）====================
 
     void resize(size_t num_joints);
 
     void setMode(size_t index, Mode mode);
     void setModes(Mode mode);
     void setModes(const std::vector<Mode>& modes);
 
     void setPosition(size_t index, float value);
     void setVelocity(size_t index, float value);
     void setEffort(size_t index, float value);
 
     void setPositions(const std::vector<float>& values);
     void setVelocities(const std::vector<float>& values);
     void setEfforts(const std::vector<float>& values);
 
     template<size_t N>
     void setPositions(const std::array<float, N>& values) {
         std::lock_guard<std::mutex> lock(mutex_);
         positions_.resize(N);
         for (size_t i = 0; i < N; ++i) positions_[i] = values[i];
     }
 
     template<size_t N>
     void setVelocities(const std::array<float, N>& values) {
         std::lock_guard<std::mutex> lock(mutex_);
         velocities_.resize(N);
         for (size_t i = 0; i < N; ++i) velocities_[i] = values[i];
     }
 
     template<size_t N>
     void setEfforts(const std::array<float, N>& values) {
         std::lock_guard<std::mutex> lock(mutex_);
         efforts_.resize(N);
         for (size_t i = 0; i < N; ++i) efforts_[i] = values[i];
     }
 
     // ==================== 获取方法（加锁）====================
 
     size_t size() const;
 
     Mode  getMode(size_t index) const;
     float getPosition(size_t index) const;
     float getVelocity(size_t index) const;
     float getEffort(size_t index) const;
     const std::string& getName(size_t index) const;
 
     // 返回副本，避免调用方持有引用期间数据被修改
     std::vector<float> getPositions() const;
     std::vector<float> getVelocities() const;
     std::vector<float> getEfforts() const;
 
     void idle();
 
     // ==================== 转换方法（加锁）====================
     msg_JointState  toMsg(size_t index) const;
     void            fromMsg(size_t index, const msg_JointState& msg);
     msg_JointStates toMsgs() const;          // 定时器线程读取
     void            fromMsgs(const msg_JointStates& msgs);
     void            print() const;
 
 private:
     mutable std::mutex  mutex_;   // mutable 允许 const 方法加锁
     std::vector<Mode>   modes_;
     std::vector<float>  positions_;
     std::vector<float>  velocities_;
     std::vector<float>  efforts_;
 };
 
 } // namespace wheel_legged_interfaces
 
 #endif // JOINT_STATE_INTERFACE_HPP