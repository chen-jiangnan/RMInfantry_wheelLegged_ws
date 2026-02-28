/**
 * @file joint_cmd_interface.cpp
 * @brief JointCmd消息的C++封装接口实现（线程安全版）
 */

 #include "wheel_legged_interfaces/joint_cmd_interface.hpp"
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
     std::lock_guard<std::mutex> lock(mutex_);
     modes_.resize(num_joints, MODE_IDLE);
     positions_.resize(num_joints, 0.0f);
     velocities_.resize(num_joints, 0.0f);
     efforts_.resize(num_joints, 0.0f);
     kps_.resize(num_joints, 0.0f);
     kds_.resize(num_joints, 0.0f);
 }
 
 void JointCmdInterface::setMode(size_t index, Mode mode) {
     std::lock_guard<std::mutex> lock(mutex_);
     if (index < modes_.size()) modes_[index] = mode;
 }
 
 void JointCmdInterface::setModes(Mode mode) {
     std::lock_guard<std::mutex> lock(mutex_);
     for (auto& m : modes_) m = mode;
 }
 
 void JointCmdInterface::setModes(const std::vector<Mode>& modes) {
     std::lock_guard<std::mutex> lock(mutex_);
     modes_ = modes;
 }
 
 void JointCmdInterface::setPosition(size_t index, float value) {
     std::lock_guard<std::mutex> lock(mutex_);
     if (index < positions_.size()) positions_[index] = value;
 }
 
 void JointCmdInterface::setVelocity(size_t index, float value) {
     std::lock_guard<std::mutex> lock(mutex_);
     if (index < velocities_.size()) velocities_[index] = value;
 }
 
 void JointCmdInterface::setEffort(size_t index, float value) {
     std::lock_guard<std::mutex> lock(mutex_);
     if (index < efforts_.size()) efforts_[index] = value;
 }
 
 void JointCmdInterface::setKp(size_t index, float value) {
     std::lock_guard<std::mutex> lock(mutex_);
     if (index < kps_.size()) kps_[index] = value;
 }
 
 void JointCmdInterface::setKd(size_t index, float value) {
     std::lock_guard<std::mutex> lock(mutex_);
     if (index < kds_.size()) kds_[index] = value;
 }
 
 void JointCmdInterface::setPositions(const std::vector<float>& values) {
     std::lock_guard<std::mutex> lock(mutex_);
     positions_ = values;
 }
 
 void JointCmdInterface::setVelocities(const std::vector<float>& values) {
     std::lock_guard<std::mutex> lock(mutex_);
     velocities_ = values;
 }
 
 void JointCmdInterface::setEfforts(const std::vector<float>& values) {
     std::lock_guard<std::mutex> lock(mutex_);
     efforts_ = values;
 }
 
 void JointCmdInterface::setKps(const std::vector<float>& values) {
     std::lock_guard<std::mutex> lock(mutex_);
     kps_ = values;
 }
 
 void JointCmdInterface::setKds(const std::vector<float>& values) {
     std::lock_guard<std::mutex> lock(mutex_);
     kds_ = values;
 }
 
 // ==================== 获取方法 ====================
 
 size_t JointCmdInterface::size() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return modes_.size();
 }
 
 JointCmdInterface::Mode JointCmdInterface::getMode(size_t index) const {
     std::lock_guard<std::mutex> lock(mutex_);
     return index < modes_.size() ? modes_[index] : MODE_IDLE;
 }
 
 float JointCmdInterface::getPosition(size_t index) const {
     std::lock_guard<std::mutex> lock(mutex_);
     return index < positions_.size() ? positions_[index] : 0.0f;
 }
 
 float JointCmdInterface::getVelocity(size_t index) const {
     std::lock_guard<std::mutex> lock(mutex_);
     return index < velocities_.size() ? velocities_[index] : 0.0f;
 }
 
 float JointCmdInterface::getEffort(size_t index) const {
     std::lock_guard<std::mutex> lock(mutex_);
     return index < efforts_.size() ? efforts_[index] : 0.0f;
 }
 
 float JointCmdInterface::getKp(size_t index) const {
     std::lock_guard<std::mutex> lock(mutex_);
     return index < kps_.size() ? kps_[index] : 0.0f;
 }
 
 float JointCmdInterface::getKd(size_t index) const {
     std::lock_guard<std::mutex> lock(mutex_);
     return index < kds_.size() ? kds_[index] : 0.0f;
 }
 
 std::vector<float> JointCmdInterface::getPositions() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return positions_;   // 返回副本
 }
 
 std::vector<float> JointCmdInterface::getVelocities() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return velocities_;
 }
 
 std::vector<float> JointCmdInterface::getEfforts() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return efforts_;
 }
 
 // ==================== 便捷方法 ====================
 
 void JointCmdInterface::idle() {
     setModes(MODE_IDLE);
 }
 
 // ==================== 转换方法 ====================
 
 JointCmdInterface::msg_JointCmd JointCmdInterface::toMsg(size_t index) const {
     std::lock_guard<std::mutex> lock(mutex_);
     msg_JointCmd msg;
     msg.mode = index < modes_.size()      ? modes_[index]      : MODE_IDLE;
     msg.q    = index < positions_.size()  ? positions_[index]  : 0.0f;
     msg.dq   = index < velocities_.size() ? velocities_[index] : 0.0f;
     msg.tau  = index < efforts_.size()    ? efforts_[index]    : 0.0f;
     msg.kp   = index < kps_.size()        ? kps_[index]        : 0.0f;
     msg.kd   = index < kds_.size()        ? kds_[index]        : 0.0f;
     return msg;
 }
 
 void JointCmdInterface::fromMsg(size_t index, const msg_JointCmd& msg) {
     std::lock_guard<std::mutex> lock(mutex_);
     if (index >= modes_.size()) return;
     modes_[index]      = static_cast<Mode>(msg.mode);
     positions_[index]  = msg.q;
     velocities_[index] = msg.dq;
     efforts_[index]    = msg.tau;
     kps_[index]        = msg.kp;
     kds_[index]        = msg.kd;
 }
 
 JointCmdInterface::msg_JointCmds JointCmdInterface::toMsgs() const {
     std::lock_guard<std::mutex> lock(mutex_);
     msg_JointCmds msgs;
     msgs.joint_cmds.reserve(modes_.size());
     for (size_t i = 0; i < modes_.size(); ++i) {
         msg_JointCmd msg;
         msg.mode = modes_[i];
         msg.q    = i < positions_.size()  ? positions_[i]  : 0.0f;
         msg.dq   = i < velocities_.size() ? velocities_[i] : 0.0f;
         msg.tau  = i < efforts_.size()    ? efforts_[i]    : 0.0f;
         msg.kp   = i < kps_.size()        ? kps_[i]        : 0.0f;
         msg.kd   = i < kds_.size()        ? kds_[i]        : 0.0f;
         msgs.joint_cmds.push_back(msg);
     }
     return msgs;
 }
 
 void JointCmdInterface::fromMsgs(const msg_JointCmds& msgs) {
     std::lock_guard<std::mutex> lock(mutex_);
     if (msgs.joint_cmds.size() != modes_.size() || msgs.joint_cmds.empty()) {
         std::cout << "msgs and joint size mismatch or msgs is empty" << std::endl;
         return;
     }
     for (size_t i = 0; i < modes_.size(); ++i) {
         const auto& jc = msgs.joint_cmds[i];
         modes_[i]      = static_cast<Mode>(jc.mode);
         positions_[i]  = jc.q;
         velocities_[i] = jc.dq;
         efforts_[i]    = jc.tau;
         kps_[i]        = jc.kp;
         kds_[i]        = jc.kd;
     }
 }
 
 void JointCmdInterface::print() const {
     std::lock_guard<std::mutex> lock(mutex_);
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