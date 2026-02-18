#include "rclcpp/rclcpp.hpp"


class ShootControlNode : public rclcpp::Node{
public:
ShootControlNode(std::string name):Node(name){
    RCLCPP_INFO(this->get_logger(), "%s节点已启动.", name.c_str());
  }
  ~ShootControlNode() {
  }
private:
};

int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<ShootControlNode>("shoot_control_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}