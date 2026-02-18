#include "rclcpp/rclcpp.hpp"


class GimbalControlNode : public rclcpp::Node{
public:
GimbalControlNode(std::string name):Node(name){
    RCLCPP_INFO(this->get_logger(), "%s节点已启动.", name.c_str());
  }
  ~GimbalControlNode() {
  }
private:
};

int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<GimbalControlNode>("gimbal_control_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}