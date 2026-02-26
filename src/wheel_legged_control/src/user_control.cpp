#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "wheel_legged_msgs/msg/chassis_ctrl.hpp"
#include "wheel_legged_msgs/msg/chassis_state.hpp"
#include "chassis_ctrl_interface.hpp"
#include "joint_fsm.hpp"



using namespace wheel_legged_interfaces;

class UserControlNode : public rclcpp::Node{
public:
UserControlNode(std::string name):Node(name){
    RCLCPP_INFO(this->get_logger(), "%s节点已启动.", name.c_str());

    // ── 接口对象 ──
    chassis_ctrl_ = std::make_unique<ChassisCtrlInterface>();

    // ── ROS2 发布/订阅 ──
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, 
        [this](sensor_msgs::msg::Joy::SharedPtr msg) {
          if(msg->buttons[3] == 1){chassis_ctrl_->setMode(JFSMode::READY);}       //Y
          if(msg->buttons[2] == 1){chassis_ctrl_->setMode(JFSMode::ZEROTAU);}     //X
          chassis_ctrl_->setVelocity(msg->axes[1]*2);
          if(msg->buttons[4] == 1){
            chassis_ctrl_->setYawSpeed(15);
          }else{
            chassis_ctrl_->setYawSpeed(msg->axes[3]*2);
          }
        });

    chassis_ctrl_pub_ = this->create_publisher<wheel_legged_msgs::msg::ChassisCtrl>(
        "highlevel/chassisCtrl", 10);
    
    // ── 定时器 ──
    chassis_ctrl_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        [this] {
        float position = chassis_ctrl_->getPosition() + chassis_ctrl_->getVelocity()*0.01; 
        chassis_ctrl_->setPosition(position);
        chassis_ctrl_pub_->publish(chassis_ctrl_->toMsg()); });      
  }
private:
    // ── 接口 ──
    std::unique_ptr<ChassisCtrlInterface> chassis_ctrl_;
    
    // ── ROS2 ──
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr            joy_sub_;
    rclcpp::Publisher<wheel_legged_msgs::msg::ChassisCtrl>::SharedPtr chassis_ctrl_pub_;
    rclcpp::TimerBase::SharedPtr chassis_ctrl_timer_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<UserControlNode>("user_control_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}