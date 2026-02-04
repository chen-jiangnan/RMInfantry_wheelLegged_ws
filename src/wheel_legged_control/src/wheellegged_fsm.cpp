#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "FSM/jointFSM.hpp"
#include "wheel_legged_msgs/msg/chassis_ctrl.hpp"
#include "wheel_legged_msgs/msg/chassis_state.hpp"
#include <rclcpp/publisher.hpp>
#include <wheel_legged_msgs/msg/detail/chassis_ctrl__struct.hpp>
#include <wheel_legged_msgs/msg/detail/chassis_state__struct.hpp>

namespace chassis_control{
  class Ctrl{
  public:
    Ctrl(){}
    float target_x[6] = {0};    
    float target_L0[2] = {0.2, 0.2};   
    float target_Roll = {0};
    float target_wYaw = {0};    
    float target_fs[2]={0}; 
  };
}


class WheelLeggedFSMNode : public rclcpp::Node{
public:
    WheelLeggedFSMNode(std::string name):Node(name){
    RCLCPP_INFO(this->get_logger(), "%s节点已启动.", name.c_str());
    fsm_ = std::make_unique<JFSM>(&event_);
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 1, std::bind(&WheelLeggedFSMNode::JoyCmd_callback, this, std::placeholders::_1));
    chassis_state_publisher_ = this->create_subscription<wheel_legged_msgs::msg::ChassisState>(
        "sport/chassisState", 1, std::bind(&WheelLeggedFSMNode::ChassisState_callback, this, std::placeholders::_1));      
    chassis_ctrl_publisher_ = this->create_publisher<wheel_legged_msgs::msg::ChassisCtrl>(
        "sport/chassisCtrl", 10);
    fsmTask_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),  // 100Hz
      std::bind(&WheelLeggedFSMNode::FSMTask_timCallback, this));
    chassisCtrl_pubTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),  // 100Hz
      std::bind(&WheelLeggedFSMNode::ChassisCtrl_timCallback, this));      
  }
private:
    void FSMTask_timCallback(){
      fsm_->JFSMUpdate();
      JumpTest_timCallback();
    }
    void JumpTest_timCallback(){
      if(jump_signal_){
        //step1:
        static int jump_state_ = 0;
        if(jump_state_ == 0){
          chassis_ctrl_.target_fs[0] = chassis_ctrl_.target_fs[1]= 0;
          chassis_ctrl_.target_L0[0] = chassis_ctrl_.target_L0[1]= 0.15;
          if(test_L0_[0] < 0.15){jump_state_++;}//判断下蹲是否完成
        }
        else if(jump_state_ == 1){
          chassis_ctrl_.target_fs[0] = chassis_ctrl_.target_fs[1]= 630;
          chassis_ctrl_.target_L0[0] = chassis_ctrl_.target_L0[1]= 0.35;
          if(test_L0_[0] > 0.35){jump_state_++;}//判断伸腿是否完成      
        }
        else if(jump_state_ == 2){
          chassis_ctrl_.target_fs[0] = chassis_ctrl_.target_fs[1]= -300;
          chassis_ctrl_.target_L0[0] = chassis_ctrl_.target_L0[1]= 0.13;
          if(test_L0_[0] < 0.13){jump_state_++;}//判断机体速度是否向下     
        }else if(jump_state_ == 3){
          chassis_ctrl_.target_fs[0] = chassis_ctrl_.target_fs[1]= 0;
          chassis_ctrl_.target_L0[0] = chassis_ctrl_.target_L0[1]= 0.2;
          jump_state_= 0;
          jump_signal_ = false;    
        }       
      }
    }
    void JoyCmd_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
      // RCLCPP_INFO(this->get_logger(), "接收到joy 话题信息");
      if(msg->buttons[4] == 1){event_.cali=1;}
      if(msg->buttons[5] == 1){event_.cali=0;}
      if(msg->buttons[3] == 1){fsm_->setReadyState();} //Y
      if(msg->buttons[2] == 1){fsm_->setResetState();} //X
      if(msg->buttons[7] == 1){fsm_->setDampingState();} //Y
      if(msg->buttons[6] == 1){fsm_->setCaliState();} //Y
      if(msg->buttons[11] == 1){fsm_->setZeroTauState();} //
      chassis_ctrl_.target_x[3] = msg->axes[1]*2;
      if(msg->buttons[0] == 1){
        jump_signal_ = true;
      }
    }
    void ChassisState_callback(const wheel_legged_msgs::msg::ChassisState::SharedPtr msg){
      test_L0_[0] = msg->fivelink_jointframe[0].l0;
      test_L0_[1] = msg->fivelink_jointframe[1].l0;
      test_acc_[0] = msg->body_worldframe.acc[0];
      test_acc_[1] = msg->body_worldframe.acc[1];
      test_acc_[2] = msg->body_worldframe.acc[2];
    }
    void ChassisCtrl_timCallback(){
      auto msg = wheel_legged_msgs::msg::ChassisCtrl();
      chassis_ctrl_.target_x[2] += chassis_ctrl_.target_x[3]*0.01;
      for(int i = 0; i<6; i++){
        msg.target_x[i] = chassis_ctrl_.target_x[i];
      }
      msg.target_x[3]=0;
      for(int i = 0; i<2; i++){
        msg.target_fs[i] = chassis_ctrl_.target_fs[i];
        msg.target_l0[i] = chassis_ctrl_.target_L0[i];    
      }
      msg.target_roll = chassis_ctrl_.target_Roll;
      chassis_ctrl_publisher_->publish(msg);
    }
  
    FSMEvent event_;
    std::unique_ptr<JFSM> fsm_;    
    rclcpp::TimerBase::SharedPtr fsmTask_timer_;
    chassis_control::Ctrl chassis_ctrl_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<wheel_legged_msgs::msg::ChassisCtrl>::SharedPtr chassis_ctrl_publisher_;
    rclcpp::Subscription<wheel_legged_msgs::msg::ChassisState>::SharedPtr chassis_state_publisher_;
    rclcpp::TimerBase::SharedPtr chassisCtrl_pubTimer_;

    bool jump_signal_ = false;
    float test_L0_[2];
    float test_acc_[3];
};

int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<WheelLeggedFSMNode>("wheellegged_fsm_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}