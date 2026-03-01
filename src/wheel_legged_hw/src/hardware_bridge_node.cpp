#include "rclcpp/rclcpp.hpp"
#include <array>
#include <cstddef>
#include <iostream>
#include <ostream>
#include <wheel_legged_msgs/msg/detail/joint_cmds__struct.hpp>
#include "dm_motor.hpp"
#include "dji_motor.hpp"
#include "hipnuc_imu.hpp"
#include "wheel_legged_interfaces/imu_state_interface.hpp"
#include "wheel_legged_interfaces/joint_state_interface.hpp"
#include "wheel_legged_interfaces/joint_cmd_interface.hpp"


using namespace wheel_legged_interfaces;
using namespace hipnuc_imu;
using namespace dm_tools;
using namespace dm_motor;
using namespace dji_motor;

DMTools* DMTools::instance_ = nullptr;


class HardwareBrigeNode : public rclcpp::Node{
public:
HardwareBrigeNode(std::string name):Node(name){
    RCLCPP_INFO(this->get_logger(), "%s节点已启动.", name.c_str());

    // ==================== 参数声明 ====================
    this->declare_parameter("imu_port", "/dev/ttyIMU");
    this->declare_parameter("imu_baudrate", 921600);

    auto imu_port = this->get_parameter("imu_port").as_string();
    auto imu_baud = this->get_parameter("imu_baudrate").as_int();


    // ==================== 初始化DMTools ================
    dm_tools_ = std::make_unique<DMTools>(DEV_USB2CANFD);
    dm_tools_->openChannel();

    // ==================== 初始化电机 ====================
    wheel_motor_group_ = std::make_unique<MotorGroupSender>(*dm_tools_, 0, 0x1FF);
    wheel_motors_.push_back(std::make_shared<M3508Motor>(0x05, *wheel_motor_group_, *dm_tools_, 268.0/17.0, true));
    wheel_motors_.push_back(std::make_shared<M3508Motor>(0x06, *wheel_motor_group_, *dm_tools_, 268.0/17.0, true));
    hip_motors_.push_back(std::make_shared<DamiaoMotor>(0x01, 0x11, *dm_tools_, DM8009, MIT_MODE));
    hip_motors_.push_back(std::make_shared<DamiaoMotor>(0x02, 0x12, *dm_tools_, DM8009, MIT_MODE));
    hip_motors_.push_back(std::make_shared<DamiaoMotor>(0x03, 0x13, *dm_tools_, DM8009, MIT_MODE));
    hip_motors_.push_back(std::make_shared<DamiaoMotor>(0x04, 0x14, *dm_tools_, DM8009, MIT_MODE));
    for(int i = 0; i < 4; i++){
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      hip_motors_[i]->enable();
    }    

    // ==================== 初始化IMU ====================
    imu_ = std::make_unique<HipnucImu>(imu_port, imu_baud);
    if (!imu_->init()) {
      RCLCPP_ERROR(this->get_logger(), "IMU初始化失败: %s", 
          imu_->getLastError().c_str());
      throw std::runtime_error("IMU初始化失败");
    }

    // ============= 创建 low level control接口 ==============
    chassis_joints_state_ = std::make_unique<JointStateInterface>(6, JointStateInterface::MODE_IDLE);
    chassis_joints_cmd_ = std::make_unique<JointCmdInterface>(6, JointCmdInterface::MODE_IDLE);

    // ==================== ROS2发布/订阅 ====================
    imu_state_pub_ = this->create_publisher<wheel_legged_msgs::msg::IMUState>(
      "lowlevel/imuState", 10);
    chassis_joints_cmd_sub_ = this->create_subscription<wheel_legged_msgs::msg::JointCmds>(
      "lowlevel/chassisJointCmd", 10, std::bind(&HardwareBrigeNode::chassisJointCmdSubCallback, this, std::placeholders::_1));     
    chassis_joints_state_pub_ = this->create_publisher<wheel_legged_msgs::msg::JointStates>(
      "lowlevel/chassisJointState", 10);

    // ==================== 创建定时器 ====================
    chassis_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),  std::bind(&HardwareBrigeNode::chassisJointStatePubCallback, this));

    //==================== 创建控制线程 ====================
    control_thread_ = std::thread(&HardwareBrigeNode::motorControlLoop, this);

    // 注册IMU回调
    imu_->setDataCallback([this](const HipnucData& data) {
      this->imuStatePubCallback(data);
    });
    imu_->start();
    RCLCPP_INFO(this->get_logger(), "IMU已启动");
    
   }

   ~HardwareBrigeNode() {
      // ---- 第一步：停止所有主动产生任务的来源 ----
      // 1. 停止控制线程（不再向电机发指令）
      if (control_thread_.joinable()) {
        control_thread_.join();
      }

      // 2. 停止定时器（不再触发 chassisJointStatePubCallback）
      chassis_timer_.reset();

      // 3. 停止 IMU 接收线程（不再触发 imuStatePubCallback）
      if (imu_) { imu_->stop(); }

      // ---- 第二步：安全关闭电机 ----
      for (int i = 0; i < 4; i++) {
          hip_motors_[i]->disable();
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      // ---- 第三步：清空电机对象（内部会 removeRecvRoute，dm_tools_ 此时仍有效）----
      hip_motors_.clear();
      wheel_motors_.clear();
      wheel_motor_group_.reset();
  }
private:

  void imuStatePubCallback(const HipnucData& data){
      // 发布自定义IMU状态（给控制器用）
    if(imu_state_pub_ != nullptr){
      auto msg = wheel_legged_msgs::msg::IMUState();
    
      msg.header.stamp = this->now();
      msg.header.frame_id = "chassis_imu";
  
      msg.quaternion[0] = data.quat[0]; // w x y z
      msg.quaternion[1] = data.quat[1]; // w x y z
      msg.quaternion[2] = data.quat[2]; // w x y z
      msg.quaternion[3] = data.quat[3]; // w x y z
  
      msg.gyroscope[0] = -data.gyro[0]; 
      msg.gyroscope[1] =  data.gyro[1];
      msg.gyroscope[2] =  data.gyro[2];
  
      msg.accelerometer[0] = -data.acc[0]; 
      msg.accelerometer[1] = data.acc[1];
      msg.accelerometer[2] = data.acc[2];
  
      msg.rpy[0] = -data.rpy[0];
      msg.rpy[1] = data.rpy[1];
      msg.rpy[2] = data.rpy[2];
      imu_state_pub_->publish(msg);
    }
  }

  void chassisJointStatePubCallback(){

    for(int i = 0; i < 2; i++){

      float front_jp = hip_motors_[1 + i*2]->getState().position;
      float front_jv = hip_motors_[1 + i*2]->getState().velocity;
      float front_jt = hip_motors_[1 + i*2]->getState().torque;

      float rear_jp = hip_motors_[0 + i*2]->getState().position;
      float rear_jv = hip_motors_[0 + i*2]->getState().velocity;
      float rear_jt = hip_motors_[0 + i*2]->getState().torque;

      float wheel_jp = wheel_motors_[0 + i]->getState().position;
      float wheel_jv = wheel_motors_[0 + i]->getState().velocity;
      float wheel_jt = wheel_motors_[0 + i]->getState().torque;

      chassis_joints_state_->setMode(0 + i*3, wheel_legged_interfaces::JointStateInterface::MODE_MIT);
      chassis_joints_state_->setPosition(0 + i*3, front_jp);
      chassis_joints_state_->setVelocity(0 + i*3, front_jv);
      chassis_joints_state_->setEffort(0 + i*3,   front_jt);

      chassis_joints_state_->setMode(1 + i*3, wheel_legged_interfaces::JointStateInterface::MODE_MIT);
      chassis_joints_state_->setPosition(1 + i*3, rear_jp);
      chassis_joints_state_->setVelocity(1 + i*3, rear_jv);
      chassis_joints_state_->setEffort(1 + i*3, rear_jt);

      chassis_joints_state_->setMode(2 + i*3, wheel_legged_interfaces::JointStateInterface::MODE_EFFORT);
      chassis_joints_state_->setPosition(2 + i*3, wheel_jp);
      chassis_joints_state_->setVelocity(2 + i*3, wheel_jv);
      chassis_joints_state_->setEffort(2 + i*3, wheel_jt);
    }
    auto msgs = chassis_joints_state_->toMsgs();
    chassis_joints_state_pub_->publish(msgs);
  }
  void chassisJointCmdSubCallback(const wheel_legged_msgs::msg::JointCmds& msgs){
    chassis_joints_cmd_->fromMsgs(msgs);
  }

  void motorControlLoop(){

    while(rclcpp::ok()){
      
      float left_wheel_jt  = chassis_joints_cmd_->getEffort(2 + 0*3);
      float right_wheel_jt = chassis_joints_cmd_->getEffort(2 + 1*3);
      wheel_motors_[0]->setTorque(left_wheel_jt);
      wheel_motors_[1]->setTorque(right_wheel_jt);
      wheel_motor_group_->send();
      for(int i = 0; i < 2; i++){

        float front_jp = chassis_joints_cmd_->getPosition(0 + i*3);
        float front_jv = chassis_joints_cmd_->getVelocity(0 + i*3);
        float front_jt = chassis_joints_cmd_->getEffort(0 + i*3);
        float front_kp = chassis_joints_cmd_->getKp(0 + i*3);
        float front_kd = chassis_joints_cmd_->getKd(0 + i*3);

        float rear_jp = chassis_joints_cmd_->getPosition(1 + i*3);
        float rear_jv = chassis_joints_cmd_->getVelocity(1 + i*3);
        float rear_jt = chassis_joints_cmd_->getEffort(1 + i*3);
        float rear_kp = chassis_joints_cmd_->getKp(1 + i*3);
        float rear_kd = chassis_joints_cmd_->getKd(1 + i*3);

        // float front_jp = 0;
        // float front_jv = 0;
        // float front_jt = 0;
        // float front_kp = 0;
        // float front_kd = 0;

        // float rear_jp = 0;
        // float rear_jv = 0;
        // float rear_jt = 0;
        // float rear_kp = 0;
        // float rear_kd = 0;


        hip_motors_[1 + i*2]->controlMIT(front_kp, front_kd, front_jp, front_jv, front_jt);
        // std::this_thread::sleep_for(std::chrono::microseconds(2));
        hip_motors_[0 + i*2]->controlMIT( rear_kp,  rear_kd,  rear_jp,  rear_jv,  rear_jt);
        // std::this_thread::sleep_for(std::chrono::microseconds(2));
        
      }
      // std::this_thread::sleep_for(std::chrono::microseconds(2000));
      
      std::this_thread::sleep_for(std::chrono::microseconds(2000));
    }
  }

    // ==================== 成员变量声明顺序 ====================
    // 声明顺序决定析构顺序（反向析构）
    // 原则：被依赖的对象必须最后声明（最先析构）
    //
    // 析构顺序（反向）：
    //   control_thread_ → chassis_timer_ → ROS发布订阅
    //   → chassis_joints → imu_
    //   → hip_motors_ → wheel_motors_ → wheel_motor_group_
    //   → dm_tools_  ← 最后析构，此时所有 removeRecvRoute 已完成

    rclcpp::TimerBase::SharedPtr                                   chassis_timer_;
    rclcpp::Publisher<wheel_legged_msgs::msg::IMUState>::SharedPtr imu_state_pub_;
    rclcpp::Publisher<wheel_legged_msgs::msg::JointStates>::SharedPtr chassis_joints_state_pub_;
    rclcpp::Subscription<wheel_legged_msgs::msg::JointCmds>::SharedPtr chassis_joints_cmd_sub_;
    std::unique_ptr<JointStateInterface>                           chassis_joints_state_;
    std::unique_ptr<JointCmdInterface>                             chassis_joints_cmd_;
    std::unique_ptr<HipnucImu>                                     imu_;
    std::vector<std::shared_ptr<DamiaoMotor>>                      hip_motors_;
    std::vector<std::shared_ptr<M3508Motor>>                       wheel_motors_;
    std::unique_ptr<MotorGroupSender>                              wheel_motor_group_;
    std::unique_ptr<DMTools>                                       dm_tools_;  // ← 最后声明，最先析构
    std::thread                                                    control_thread_;

};

int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<HardwareBrigeNode>("hardware_brige_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}