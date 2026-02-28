/**
 * @file joint_cmd_interface.hpp
 * @brief JointCommand消息的C++封装接口（线程安全版）
 */

 #ifndef JOINT_COMMAND_INTERFACE_HPP
 #define JOINT_COMMAND_INTERFACE_HPP
 
 #include <array>
 #include <vector>
 #include <string>
 #include <mutex>
 #include "wheel_legged_msgs/msg/joint_cmd.hpp"
 #include "wheel_legged_msgs/msg/joint_cmds.hpp"
 
 namespace wheel_legged_interfaces {
 
 class JointCmdInterface {
 public:
     using msg_JointCmd  = wheel_legged_msgs::msg::JointCmd;
     using msg_JointCmds = wheel_legged_msgs::msg::JointCmds;
 
     enum Mode : uint8_t {
         MODE_IDLE     = msg_JointCmd::MODE_IDLE,
         MODE_POSITION = msg_JointCmd::MODE_POSITION,
         MODE_VELOCITY = msg_JointCmd::MODE_VELOCITY,
         MODE_EFFORT   = msg_JointCmd::MODE_EFFORT,
         MODE_MIT      = msg_JointCmd::MODE_MIT
     };
 
     explicit JointCmdInterface(size_t num_joints = 0,
                                Mode default_mode = MODE_IDLE);
 
     // ==================== 设置方法（加锁）====================
 
     void resize(size_t num_joints);
 
     void setMode(size_t index, Mode mode);
     void setModes(Mode mode);
     void setModes(const std::vector<Mode>& modes);
 
     void setPosition(size_t index, float value);
     void setVelocity(size_t index, float value);
     void setEffort(size_t index, float value);
     void setKp(size_t index, float value);
     void setKd(size_t index, float value);
 
     void setPositions(const std::vector<float>& values);
     void setVelocities(const std::vector<float>& values);
     void setEfforts(const std::vector<float>& values);
     void setKps(const std::vector<float>& values);
     void setKds(const std::vector<float>& values);
 
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
     float getKp(size_t index) const;
     float getKd(size_t index) const;
     const std::string& getName(size_t index) const;
 
     // 返回副本，避免调用方持有引用期间数据被修改
     std::vector<float> getPositions() const;
     std::vector<float> getVelocities() const;
     std::vector<float> getEfforts() const;
 
     void idle();
 
     // ==================== 转换方法（加锁）====================
     msg_JointCmd  toMsg(size_t index) const;
     void          fromMsg(size_t index, const msg_JointCmd& msg);
     msg_JointCmds toMsgs() const;
     void          fromMsgs(const msg_JointCmds& msgs);  // ROS回调线程写入
     void          print() const;
 
 private:
     mutable std::mutex  mutex_;   // mutable 允许 const 方法加锁
     std::vector<Mode>   modes_;
     std::vector<float>  positions_;
     std::vector<float>  velocities_;
     std::vector<float>  efforts_;
     std::vector<float>  kps_;
     std::vector<float>  kds_;
 };
 
 } // namespace wheel_legged_interfaces
 
 #endif // JOINT_COMMAND_INTERFACE_HPP