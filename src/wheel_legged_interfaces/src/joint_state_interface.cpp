/**
 * @file joint_state_interface.cpp
 * @brief JointState消息的C++封装接口实现（线程安全版）
 */

 #include "wheel_legged_interfaces/joint_state_interface.hpp"
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
     std::lock_guard<std::mutex> lock(mutex_);
     modes_.resize(num_joints, MODE_IDLE);
     positions_.resize(num_joints, 0.0f);
     velocities_.resize(num_joints, 0.0f);
     efforts_.resize(num_joints, 0.0f);
 }
 
 void JointStateInterface::setMode(size_t index, Mode mode) {
     std::lock_guard<std::mutex> lock(mutex_);
     if (index < modes_.size()) modes_[index] = mode;
 }
 
 void JointStateInterface::setModes(Mode mode) {
     std::lock_guard<std::mutex> lock(mutex_);
     for (auto& m : modes_) m = mode;
 }
 
 void JointStateInterface::setModes(const std::vector<Mode>& modes) {
     std::lock_guard<std::mutex> lock(mutex_);
     modes_ = modes;
 }
 
 void JointStateInterface::setPosition(size_t index, float value) {
     std::lock_guard<std::mutex> lock(mutex_);
     if (index < positions_.size()) positions_[index] = value;
 }
 
 void JointStateInterface::setVelocity(size_t index, float value) {
     std::lock_guard<std::mutex> lock(mutex_);
     if (index < velocities_.size()) velocities_[index] = value;
 }
 
 void JointStateInterface::setEffort(size_t index, float value) {
     std::lock_guard<std::mutex> lock(mutex_);
     if (index < efforts_.size()) efforts_[index] = value;
 }
 
 void JointStateInterface::setPositions(const std::vector<float>& values) {
     std::lock_guard<std::mutex> lock(mutex_);
     positions_ = values;
 }
 
 void JointStateInterface::setVelocities(const std::vector<float>& values) {
     std::lock_guard<std::mutex> lock(mutex_);
     velocities_ = values;
 }
 
 void JointStateInterface::setEfforts(const std::vector<float>& values) {
     std::lock_guard<std::mutex> lock(mutex_);
     efforts_ = values;
 }
 
 // ==================== 获取方法 ====================
 
 size_t JointStateInterface::size() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return modes_.size();
 }
 
 JointStateInterface::Mode JointStateInterface::getMode(size_t index) const {
     std::lock_guard<std::mutex> lock(mutex_);
     return index < modes_.size() ? modes_[index] : MODE_IDLE;
 }
 
 float JointStateInterface::getPosition(size_t index) const {
     std::lock_guard<std::mutex> lock(mutex_);
     return index < positions_.size() ? positions_[index] : 0.0f;
 }
 
 float JointStateInterface::getVelocity(size_t index) const {
     std::lock_guard<std::mutex> lock(mutex_);
     return index < velocities_.size() ? velocities_[index] : 0.0f;
 }
 
 float JointStateInterface::getEffort(size_t index) const {
     std::lock_guard<std::mutex> lock(mutex_);
     return index < efforts_.size() ? efforts_[index] : 0.0f;
 }
 
 std::vector<float> JointStateInterface::getPositions() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return positions_;   // 返回副本
 }
 
 std::vector<float> JointStateInterface::getVelocities() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return velocities_;
 }
 
 std::vector<float> JointStateInterface::getEfforts() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return efforts_;
 }
 
 // ==================== 便捷方法 ====================
 
 void JointStateInterface::idle() {
     setModes(MODE_IDLE);
 }
 
 // ==================== 转换方法 ====================
 
 JointStateInterface::msg_JointState JointStateInterface::toMsg(size_t index) const {
     std::lock_guard<std::mutex> lock(mutex_);
     msg_JointState msg;
     msg.mode = index < modes_.size()      ? modes_[index]      : MODE_IDLE;
     msg.q    = index < positions_.size()  ? positions_[index]  : 0.0f;
     msg.dq   = index < velocities_.size() ? velocities_[index] : 0.0f;
     msg.tau  = index < efforts_.size()    ? efforts_[index]    : 0.0f;
     return msg;
 }
 
 void JointStateInterface::fromMsg(size_t index, const msg_JointState& msg) {
     std::lock_guard<std::mutex> lock(mutex_);
     if (index >= modes_.size()) return;
     modes_[index]      = static_cast<Mode>(msg.mode);
     positions_[index]  = msg.q;
     velocities_[index] = msg.dq;
     efforts_[index]    = msg.tau;
 }
 
 JointStateInterface::msg_JointStates JointStateInterface::toMsgs() const {
     std::lock_guard<std::mutex> lock(mutex_);
     msg_JointStates msgs;
     msgs.joint_states.reserve(modes_.size());
     for (size_t i = 0; i < modes_.size(); ++i) {
         msg_JointState msg;
         msg.mode = modes_[i];
         msg.q    = i < positions_.size()  ? positions_[i]  : 0.0f;
         msg.dq   = i < velocities_.size() ? velocities_[i] : 0.0f;
         msg.tau  = i < efforts_.size()    ? efforts_[i]    : 0.0f;
         msgs.joint_states.push_back(msg);
     }
     return msgs;
 }
 
 void JointStateInterface::fromMsgs(const msg_JointStates& msgs) {
     std::lock_guard<std::mutex> lock(mutex_);
     if (msgs.joint_states.size() != modes_.size() || msgs.joint_states.empty()) {
         std::cout << "msgs and joint size mismatch or msgs is empty" << std::endl;
         return;
     }
     for (size_t i = 0; i < modes_.size(); ++i) {
         const auto& js = msgs.joint_states[i];
         modes_[i]      = static_cast<Mode>(js.mode);
         positions_[i]  = js.q;
         velocities_[i] = js.dq;
         efforts_[i]    = js.tau;
     }
 }
 
 void JointStateInterface::print() const {
     std::lock_guard<std::mutex> lock(mutex_);
     std::cout << std::fixed << std::setprecision(4);
     std::cout << "\n========== JointState (" << modes_.size() << " joints) ==========\n";
     std::cout << std::setw(4)  << "idx"
               << std::setw(10) << "mode"
               << std::setw(10) << "pos"
               << std::setw(10) << "vel"
               << std::setw(10) << "eff" << "\n";
     std::cout << std::string(44, '-') << "\n";
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
                   << "\n";
     }
     std::cout << "==============================================\n";
 }
 
 } // namespace wheel_legged_interfaces