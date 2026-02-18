#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "wheel_legged_msgs/msg/chassis_ctrl.hpp"
#include "wheel_legged_msgs/msg/chassis_state.hpp"
#include "chassis_ctrl_interface.hpp"



using namespace wheel_legged_interfaces;

class UserControlNode : public rclcpp::Node{
public:
UserControlNode(std::string name):Node(name){
    RCLCPP_INFO(this->get_logger(), "%s节点已启动.", name.c_str());

    // ==================== 创建 High level control接口 =================
    chassis_ctrl_ = std::make_unique<ChassisCtrlInterface>();
    // ==================== 创建ROS2订阅/发布者 ==================
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, std::bind(&UserControlNode::joySubCallback, this, std::placeholders::_1));     
    chassis_ctrl_pub_ = this->create_publisher<wheel_legged_msgs::msg::ChassisCtrl>(
        "sport/chassisCtrl", 10);
    // ==================== 创建定时器 ==================
    chassis_ctrl_pubTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&UserControlNode::chassisCtrlPubCallback, this));      
  }
private:
    void joySubCallback(const sensor_msgs::msg::Joy& msg){
    // RCLCPP_INFO(this->get_logger(), "接收到joy 话题信息");
    //   if(msg.buttons[4] == 1){event_.cali=1;}
    //   if(msg.buttons[5] == 1){event_.cali=0;}
    //   if(msg.buttons[3] == 1){fsm_.setReadyState();}     //Y
    //   if(msg.buttons[2] == 1){fsm_.setResetState();}     //X
    //   if(msg.buttons[7] == 1){fsm_.setDampingState();}   //Y
    //   if(msg.buttons[6] == 1){fsm_.setCaliState();}      //Y
    //   if(msg.buttons[11] == 1){fsm_->setZeroTauState();} //
      chassis_ctrl_->setVelocity(msg.axes[1]*2);
      if(msg.buttons[4] == 1){
        chassis_ctrl_->setYawSpeed(15);
      }else{
        chassis_ctrl_->setYawSpeed(msg.axes[3]*2);
      }
    //   if(msg->buttons[0] == 1){
    //     jump_signal_ = true;
    //   }
    }

    void chassisCtrlPubCallback(){
        float position = chassis_ctrl_->getPosition() + chassis_ctrl_->getVelocity()*0.01;
        chassis_ctrl_->setPosition(position);
        auto msg = chassis_ctrl_->toMsg();
        chassis_ctrl_pub_->publish(msg);
    }

    void controlLoop(){
        while(rclcpp::ok()){

        }
    }

    /*类对象*/
    std::unique_ptr<ChassisCtrlInterface> chassis_ctrl_;
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<wheel_legged_msgs::msg::ChassisCtrl>::SharedPtr chassis_ctrl_pub_;
    rclcpp::TimerBase::SharedPtr chassis_ctrl_pubTimer_;

    /*控制线程对象*/
};

int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<UserControlNode>("user_control_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}