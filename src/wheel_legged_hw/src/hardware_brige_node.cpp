#include "rclcpp/rclcpp.hpp"
#include <array>
#include <cstddef>
#include <iostream>
#include <ostream>
#include "dm_motor.hpp"
#include "dji_motor.hpp"


void canSentCallback(usb_rx_frame_t* frame){
  printf("can sent callback , packet id:%x\n",frame->head.can_id);
  // switch (condition) {
  // cases
  // }

}
void canRecvCallback(usb_rx_frame_t* frame){
  printf("can recv callback , packet id:%x\n",frame->head.can_id);
}

class HardwareBrigeNode : public rclcpp::Node{
public:
HardwareBrigeNode(std::string name):Node(name){
    RCLCPP_INFO(this->get_logger(), "%s节点已启动.", name.c_str());
    // ==================== 创建USB2FDCAN工具 ====================
    //初始化模块句柄
    dm_tools_ = damiao_handle_create(DEV_USB2CANFD);
    RCLCPP_INFO(this->get_logger(), "初始化 usb2fdcan 设备...");

    //打印sdk版本信息
    int device_cnt = damiao_handle_find_devices(dm_tools_);
    if (device_cnt == 0) { 
      RCLCPP_ERROR(this->get_logger(), "未发现 usb2fdcan 设备! ! !");
      throw std::runtime_error("usb2fdcan 设备初始化失败");
    }

    //查找对应类型模块的设备数量
    int handle_cnt = 0;
    damiao_handle_get_devices(dm_tools_, dev_list_, &handle_cnt);
    
    //打开设备
    if (device_open(dev_list_[0])) {
      RCLCPP_INFO(this->get_logger(), "usb2fdcan 设备已开启");
    }else{
      RCLCPP_ERROR(this->get_logger(), "usb2fdcan 设备开启失败! ! !");
      throw std::runtime_error("usb2fdcan 设备初始化失败");
    }

    //获取设备信息
    char buf[255];
    device_get_version(dev_list_[0], buf, sizeof(buf));
    RCLCPP_INFO(this->get_logger(), "device version:%s", buf);

    device_get_serial_number(dev_list_[0], buf, sizeof(buf));
    RCLCPP_INFO(this->get_logger(), "device sn:%s", buf);

    //设置通道波特率
    device_channel_set_baud_with_sp(dev_list_[0], 0, false, 1000000, 1000000, 0.75f, 0.75f);
    device_baud_t baud;
    //获取通道波特率
    if (device_channel_get_baudrate(dev_list_[0], 0, &baud))
    {
      RCLCPP_INFO(this->get_logger(), "=== CAN channel baudrate info ===");
      RCLCPP_INFO(this->get_logger(), "can_baud: %d", baud.can_baudrate);
      RCLCPP_INFO(this->get_logger(), "canfd_baud: %d", baud.canfd_baudrate);
      RCLCPP_INFO(this->get_logger(), "can_sp: %.6f", baud.can_sp);
      RCLCPP_INFO(this->get_logger(), "canfd_sp: %.6f", baud.canfd_sp);
    }
    //开启can通道
    auto test = device_open_channel(dev_list_[0], 0);
    if(!test){
      RCLCPP_ERROR(this->get_logger(), "usb2fdcan 设备开启失败! ! !");
    }

    //发送钩子函数注册
    device_hook_to_sent(dev_list_[0], canSentCallback);
    //接收钩子函数注册
    device_hook_to_rec(dev_list_[0], canRecvCallback);

    // ==================== 创建电机 ====================
    hip_motors_.emplace_back(0x01, 0x01 + 0x10, dev_list_[0], damiao::DM8009, damiao::MIT_MODE);
    hip_motors_.emplace_back(0x02, 0x02 + 0x10, dev_list_[0], damiao::DM8009, damiao::MIT_MODE);
    hip_motors_.emplace_back(0x03, 0x03 + 0x10, dev_list_[0], damiao::DM8009, damiao::MIT_MODE);
    hip_motors_.emplace_back(0x04, 0x04 + 0x10, dev_list_[0], damiao::DM8009, damiao::MIT_MODE);
    for(int i = 0; i < 4; i++){ 
      hip_motors_[i].enable();
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    wheel_motors_.emplace_back(0x01, 0x02, dev_list_[0]);

    // ==================== 创建控制线程 ====================
    control_thread_ = std::thread(&HardwareBrigeNode::controlLoop, this);
   }

~HardwareBrigeNode() {
    for(int i = 0; i < 4; i++){
      hip_motors_[i].disable();
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    wheel_motors_[0].disable();
    device_hook_to_sent(dev_list_[0], NULL);
    device_hook_to_rec(dev_list_[0], NULL);
    device_close_channel(dev_list_[0],0); //关闭can通道
    device_close(dev_list_[0]);          //关闭设备
    damiao_handle_destroy(dm_tools_);   //销毁模块句柄
  }
private:

  void controlLoop(){

    while(rclcpp::ok()){
      wheel_motors_[0].controlForce(1, 1);
      std::this_thread::sleep_for(std::chrono::microseconds(2000));
    }
  }
  /*USB2FDCAN tools*/
  damiao_handle* dm_tools_;
  device_handle* dev_list_[16];
  /*chassis motor*/
  std::vector<damiao::DamiaoMotor> hip_motors_;
  std::vector<dji::WheelMotors> wheel_motors_;

  /*控制线程*/
  std::thread control_thread_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<HardwareBrigeNode>("hardware_brige_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}