#include "rclcpp/rclcpp.hpp"
#include "chassis_config_manager.hpp"
#include "Controller/LegController.hpp"
#include "Controller/LQRController.hpp"
#include "Controller/RotateController.hpp"
#include "Controller/StateEstimator.hpp"
#include "Model/WheelLeggedRobot.hpp"
#include "joint_state_interface.hpp"
#include "joint_cmd_interface.hpp"
#include "imu_state_interface.hpp"
#include "jointFSM.hpp"
#include "wheel_legged_msgs/msg/chassis_state.hpp"
#include <array>
#include <memory>
#include "chassis_ctrl_interface.hpp"
#include "chassis_state_interface.hpp"



using namespace robot;
using namespace controller;
using namespace config_manager;
using namespace wheel_legged_interfaces;

class ChassisControlNode : public rclcpp::Node{
public:
  ChassisControlNode(std::string name):Node(name){
    RCLCPP_INFO(this->get_logger(), "%s节点已启动.", name.c_str());

    // ==================== 创建配置管理器 ====================
    config_manager_ = std::make_unique<ChassisConfigManager>(this);
    // 声明参数
    config_manager_->declareParameters();    
    // 加载模型参数
    std::string model_path = this->get_parameter("model_config_path").as_string();
    if (!config_manager_->loadModelParameters(model_path)) {
        throw std::runtime_error("模型参数加载失败");
    }
    // 加载控制器参数
    config_manager_->loadControllerParameters();
    config_manager_->printConfiguration();
    config_manager_->setParameterCallback(
      std::bind(&ChassisControlNode::onParamChanged, 
               this, std::placeholders::_1)
    );

    // ==================== 创建 High level control接口 =================
    // imu 接口  
    imu_state_ = std::make_unique<IMUStateInterface>();
    // 关节 state/cmd 接口
    joints_state_ = std::make_unique<JointStateInterface>(6, JointStateInterface::MODE_IDLE);
    joints_state_->setNames(robot_.join_names);
    joints_cmd_ = std::make_unique<JointCmdInterface>(6, JointCmdInterface::MODE_IDLE);
    joints_cmd_->setNames(robot_.join_names);
    // 底盘 state/ctrl 接口
    chassis_ctrl_ = std::make_unique<ChassisCtrlInterface>();
    chassis_state_ = std::make_unique<ChassisStateInterface>();
    chassis_state_->bind(&robot_, &leg_controller_, &lqr_controller_, &rotate_controller_, &state_estimator_);

    // ==================== 创建ROS2订阅/发布者 ==================
    imu_state_sub_ = this->create_subscription<wheel_legged_msgs::msg::IMUState>(
      "simulation/IMUState", 10, std::bind(&ChassisControlNode::imuStateSubCallback, this, std::placeholders::_1));   
    joints_state_sub_ = this->create_subscription<wheel_legged_msgs::msg::JointStates>(
      "simulation/chassisMotorState", 10, std::bind(&ChassisControlNode::jointStateSubCallback, this, std::placeholders::_1));     
    joints_cmd_pub_ = this->create_publisher<wheel_legged_msgs::msg::JointCmds>(
      "simulation/chassisMotorCmd", 10);
    chassis_ctrl_sub_ = this->create_subscription<wheel_legged_msgs::msg::ChassisCtrl>(
      "sport/chassisCtrl", 10, std::bind(&ChassisControlNode::chassisCtrlSubCallback, this, std::placeholders::_1));  
    chassis_state_pub_ = this->create_publisher<wheel_legged_msgs::msg::ChassisState>(
      "sport/chassisState", 10);
      
    // ==================== 创建定时器 ==================
    joint_cmd_pubTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),  std::bind(&ChassisControlNode::jointCmdPubCallback, this));
    chassis_state_pubTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),  std::bind(&ChassisControlNode::chassisStatePubCallback, this));
    debug_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),  std::bind(&ChassisControlNode::debugCallback, this));


    // chassis_state_pubTimer_ = this->create_wall_timer(
    //   std::chrono::milliseconds(1),  // 1000Hz
    //   std::bind(&ChassisControlNode::ChassisState_timCallback, this));
    
    // ==================== 创建任务线程 ====================
    control_thread_ = std::thread(&ChassisControlNode::controlLoop, this);

  }
  ~ChassisControlNode() {
    if (control_thread_.joinable()) {
      control_thread_.join();
    } 
  }
private:
  void onParamChanged(const std::vector<rclcpp::Parameter>& params){
    RCLCPP_INFO(this->get_logger(), "参数变化,重新配置控制器");
    params_changed_ = true;
  }
  void reCfgControllers(){
    std::lock_guard<std::mutex> lock(param_mutex_);
        
    auto& ctrl_params = config_manager_->getControllerParams();
    
    lqr_controller_.setConfig(ctrl_params.lqr);
    leg_controller_.setConfig(ctrl_params.leg_control);
    rotate_controller_.setConfig(ctrl_params.rotate);
    state_estimator_.setConfig(ctrl_params.state_estimator);
    
    RCLCPP_INFO(this->get_logger(), "所有控制器重新配置完成");    
  }
  // ==================== 回调函数 ====================
  void imuStateSubCallback(const wheel_legged_msgs::msg::IMUState& msg){
    imu_state_->fromMsg(msg);
  }

  void jointStateSubCallback(const wheel_legged_msgs::msg::JointStates& msgs){
    joints_state_->fromMsgs(msgs);
  }

  void chassisCtrlSubCallback(const wheel_legged_msgs::msg::ChassisCtrl& msg){
    chassis_ctrl_->fromMsg(msg);
  }

  void jointCmdPubCallback(){
    auto msgs = joints_cmd_->toMsgs();
    joints_cmd_pub_->publish(msgs);
  }
  void chassisStatePubCallback(){
    auto msg = chassis_state_->toMsg();
    chassis_state_pub_->publish(msg);
  }

  void debugCallback(){
    // robot_.printModelStateInfo();
    // lqr_controller_.printDebugInfo();
  }

  void jointFSMCallback(){
    jfsm_->JFSMUpdate();
  }

  void jointZeroTau(){
    // 清空控制
    leg_controller_.clear();
    rotate_controller_.clear();
    state_estimator_.clear();

    // 关节空闲状态（电机失能）
    for(int i = 0; i < 6; i++){
      joints_cmd_->setMode(i, JointCmdInterface::MODE_IDLE);
      joints_cmd_->setPosition(i, 0);
      joints_cmd_->setVelocity(i, 0);
      joints_cmd_->setEffort(i, 0);
      joints_cmd_->setKp(i, 0);
      joints_cmd_->setKd(i, 0);
    }
  }

  void jointDamping(){

    leg_controller_.clear();
    rotate_controller_.clear();
    state_estimator_.clear();

    for(int i = 0; i < 6; i++){
      joints_cmd_->setMode(i, JointCmdInterface::MODE_MIT);
      joints_cmd_->setPosition(i, 0);
      joints_cmd_->setVelocity(i, 0);
      joints_cmd_->setEffort(i, 0);
      joints_cmd_->setKp(i, 0);
      joints_cmd_->setKd(i, 3);
    } 
  }

  void jointCali(){

    leg_controller_.clear();
    rotate_controller_.clear();
    state_estimator_.clear();

    // 设置关节为MIT模式， 给髋关节固定施加转速
    // 判断关节是否到达限位
    // 设置髋关节 hip_joint_offset 零点
  }

  void jointReset(){

    leg_controller_.clear();
    rotate_controller_.clear();
    state_estimator_.clear();

    /*update body state*/
    robot_.updateBodyPose(imu_state_->getRPY(), imu_state_->getGyroscope(), imu_state_->getAccelerometer());
    /*update joint state*/
    robot_.updateAllJointState(*joints_state_);
    /*forward kinematics*/
    std::array<float, 2> rev_phi1, rev_phi4, rev_phi0, rev_L0;
    rev_phi1[0] = robot_.getHipJoints()[0].q;		// joint space map to fivelink joint frame
    rev_phi4[0] = robot_.getHipJoints()[1].q;
    rev_phi1[1] = robot_.getHipJoints()[2].q;
    rev_phi4[1] = robot_.getHipJoints()[3].q;
    robot_.forwardKinematics(rev_phi1, rev_phi4, rev_phi0, rev_L0, true);

    /**/
    std::array<float, 2> set_phi0, set_L0;
    for(int i = 0; i < 2; i++){
      // 第一阶段: 伸展腿并使腿向后旋转到目标位置
      // 第二阶段: 将腿缩回并使腿旋转到目标位置
      // 第三阶段: Reset模式结束
    }
    std::array<float, 2> set_phi1, set_phi4;
    robot_.inverseKinematics(set_phi0, set_L0, set_phi1, set_phi4);
    /**/

  }

  void jointReady(float dt){
      /*update body state*/
      robot_.updateBodyPose(imu_state_->getRPY(), imu_state_->getGyroscope(), imu_state_->getAccelerometer());
      /*update joint state*/
      robot_.updateAllJointState(*joints_state_);
      /*updata remote control data*/
      
      /*forward kinematics*/
      std::array<float, 2> rev_phi1, rev_phi4, rev_phi0, rev_L0;
      rev_phi1[0] = robot_.getHipJoints()[0].q;		// joint space map to fivelink joint frame
      rev_phi4[0] = robot_.getHipJoints()[1].q;
      rev_phi1[1] = robot_.getHipJoints()[2].q;
      rev_phi4[1] = robot_.getHipJoints()[3].q;
      robot_.forwardKinematics(rev_phi1, rev_phi4, rev_phi0, rev_L0, true);

      /*forward dynamics*/
      std::array<float, 2> rev_torqueA, rev_torqueE, rev_Tp, rev_F;
      rev_torqueA[0] = robot_.getHipJoints()[0].tau;
      rev_torqueE[0] = robot_.getHipJoints()[1].tau;
      rev_torqueA[1] = robot_.getHipJoints()[2].tau;
      rev_torqueE[1] = robot_.getHipJoints()[3].tau;
      robot_.forwardDynamics(rev_torqueA, rev_torqueE, rev_Tp, rev_F, true);

      /**/
      std::array<float, 2> rev_alpha, rev_theta;
      rev_alpha[0] = robot_.getVMCJointFrames()[0].alpha;
      rev_alpha[1] = robot_.getVMCJointFrames()[1].alpha;
      rev_theta[0] = robot_.getVMCJointFrames()[0].theta;
      rev_theta[1] = robot_.getVMCJointFrames()[1].theta;   
      state_estimator_.update(rev_alpha, rev_theta, rev_L0);

      /*XV Estimate*/
      std::array<float, 2> w_ecd;
      w_ecd[0] = robot_.getWheelJoints()[0].dq;
      w_ecd[1] = robot_.getWheelJoints()[1].dq;;
      state_estimator_.estimateVelocity(
        w_ecd, 
        robot_.getBodyWorldFrame().a[0],
        robot_.getBodyWorldFrame().rpy[1],
        robot_.getConfig().wheel_link[0].lengthOrRadius
      );

      /*Fn Estimate*/
      state_estimator_.estimateForce(
        rev_F,
        rev_Tp,
        robot_.getBodyWorldFrame().a[2] - 9.8,
        robot_.getConfig().wheel_link[0].mass
      );

      /*lqr Control*/
      LQRController6x2::StateVector lqr_xd;
      std::array<LQRController6x2::StateVector, 2> lqr_x;
      for(size_t index = 0; index < 2; index++){
        lqr_x[index](0) = robot_.getVMCJointFrames()[index].theta; 
        lqr_x[index](1) = (
           robot_.getVMCJointFrames()[index].theta_history[0] 
          -robot_.getVMCJointFrames()[index].theta_history[1]
        )/dt; 
        lqr_x[index](2) = state_estimator_.getVelocityEstimate().x_filter;
        lqr_x[index](3) = state_estimator_.getVelocityEstimate().v_filter;
        lqr_x[index](4) = -robot_.getBodyWorldFrame().rpy[1];
        lqr_x[index](5) = -robot_.getBodyWorldFrame().w[1];
        /*load lqr k table*/
        lqr_controller_.updateKFormLegLength(index, rev_L0[index]);
        if(0 && state_estimator_.detectGround()){
          // lqr_x[index](0) = 0;  //chassis_ctrl_.target_x[2];
          // lqr_x[index](1) = 0;  //chassis_ctrl_.target_x[3];
          lqr_x[index](2) = 0;  //chassis_ctrl_.target_x[2];
          lqr_x[index](3) = 0;  //chassis_ctrl_.target_x[3];
          state_estimator_.corverVelocityEstimate(0, 0);
          
          // lqr_controller_.setGainMatrix(index, 0, 0, 0);
          // lqr_controller_.setGainMatrix(index, 0, 1, 0);
          lqr_controller_.setGainMatrix(index, 0, 2, 0);
          lqr_controller_.setGainMatrix(index, 0, 3, 0);
          // lqr_controller_.setGainMatrix(index, 0, 4, 0);
          // lqr_controller_.setGainMatrix(index, 0, 5, 0);
          // lqr_controller_.setGainMatrix(index, 1, 0, 0);
          // lqr_controller_.setGainMatrix(index, 1, 1, 0);
          lqr_controller_.setGainMatrix(index, 1, 2, 0);
          lqr_controller_.setGainMatrix(index, 1, 3, 0);
          // lqr_controller_.setGainMatrix(index, 1, 4, 0);
          // lqr_controller_.setGainMatrix(index, 1, 5, 0);
        }else{

        }
        
      }
      auto lqr_u = lqr_controller_.calculate(lqr_xd, lqr_x);

      /*leg control*/
      std::array<float, 2> target_L0 = {0.20, 0.20};
      auto leg_u = leg_controller_.calculate(
        target_L0, 
        rev_L0, 
        0, 
        robot_.getBodyWorldFrame().rpy[0], 
        rev_theta, 
        rev_phi0,
        robot_.getConfig().body_link.mass*G
      );

      /*body yaw rotate control*/
      std::array<float, 2> rotate_t = {0};  		// [PID]
      // rotate_t[0] = rotate_controller_.calculate(0, robot_.getBodyWorldFrame().w[2]);
      // rotate_t[1] = -rotate_t[0];

      /*inverse dynamics*/
      std::array<float, 2> set_Tp, set_F, set_T, set_torqueA, set_torqueE;
      for(int i = 0; i < 2 ;i++)
      {

        set_Tp[i] = lqr_u[i](1) + leg_u.compensation_tp[i];
        set_F[i]	= leg_u.F[i];
        set_T[i] = lqr_u[i](0) + rotate_t[i];
      }
      robot_.inverseDynamics(set_Tp, set_F, set_torqueA, set_torqueE);


      /*PT-Mix Control*/
      // limit the q-v-t value
      for(int i = 0; i < 2; i++){
        if(set_torqueA[i] > 50.0f){
          set_torqueA[i] = 50.0f;
        }else if(set_torqueA[i] <-50.0f){
          set_torqueA[i] = -50.0f;
        }
        if(set_torqueE[i] > 50.0f){
          set_torqueE[i] = 50.0f;
        }else if(set_torqueE[i] <-50.0f){
          set_torqueE[i] = -50.0f;
        }

        if(set_T[i] > 5.0f){
          set_T[i] = 5.0f;
        }else if(set_T[i] <-5.0f){
          set_T[i] = -5.0f;
        }
      }


      std::array<float, 6> set_joints_p = {0};
      std::array<float, 6> set_joints_v = {0};
      std::array<float, 6> set_joints_t = {
        set_torqueA[0],
        set_torqueE[0],
        set_T[0],
        set_torqueA[1],
        set_torqueE[1],
        set_T[1],
      };
      for(int i = 0; i < 6; i++){

        set_joints_p[i] = (robot_.getConfig().joints[i].invert_pos ? -1 : 1) * (set_joints_p[i] - robot_.getConfig().joints[i].pos_offset);
        set_joints_v[i] = (robot_.getConfig().joints[i].invert_vel ? -1 : 1) * set_joints_v[i];
        set_joints_t[i] = (robot_.getConfig().joints[i].invert_torque ? -1 : 1) * set_joints_t[i];
        
        joints_cmd_->setMode(i, JointCmdInterface::MODE_MIT);
        joints_cmd_->setPosition(i, set_joints_p[i]);
        joints_cmd_->setVelocity(i, set_joints_v[i]);
        joints_cmd_->setEffort(i, set_joints_t[i]);
        joints_cmd_->setKp(i, 0);
        joints_cmd_->setKd(i, 0);
      }    
  }


  void controlLoop2(){
    /*初始化模型*/
    robot_.setConfig(config_manager_->getModelParams());
    /*初始化控制器*/
    auto& ctrl_params = config_manager_->getControllerParams();
    lqr_controller_.setConfig(ctrl_params.lqr);
    leg_controller_.setConfig(ctrl_params.leg_control);
    rotate_controller_.setConfig(ctrl_params.rotate);
    state_estimator_.setConfig(ctrl_params.state_estimator);
    RCLCPP_INFO(this->get_logger(), "所有控制器配置完成"); 

    while(rclcpp::ok()){

      if (params_changed_.load()) {
        reCfgControllers();
        params_changed_.store(false);
      }

      double dt = 1.0 / ctrl_params.control_frequency;
      switch(jfsm_->getCurrentState()){
        case JFSMode::ZEROTAU:  jointZeroTau();  break;
        case JFSMode::CALI:     jointCali();     break;
        case JFSMode::READY:    jointReady(dt);  break;
        case JFSMode::RESET:    jointReset();    break;
        case JFSMode::DAMPING:  jointDamping();  break;
      };

      std::chrono::duration<double> period(dt);
      std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::microseconds>(period));
    }
  }

  // ==================== 控制循环 ====================
  void controlLoop(){
    /*初始化模型*/
    robot_.setConfig(config_manager_->getModelParams());
    /*初始化控制器*/
    auto& ctrl_params = config_manager_->getControllerParams();
    lqr_controller_.setConfig(ctrl_params.lqr);
    leg_controller_.setConfig(ctrl_params.leg_control);
    rotate_controller_.setConfig(ctrl_params.rotate);
    state_estimator_.setConfig(ctrl_params.state_estimator);
    RCLCPP_INFO(this->get_logger(), "所有控制器配置完成"); 

    std::array<float, 2> target_L0 = {0.20, 0.20};

    while(rclcpp::ok()){
      /**/
      if (params_changed_.load()) {
        reCfgControllers();
        params_changed_.store(false);
      }

      double period_sec = 1.0 / ctrl_params.control_frequency;

      /*update body state*/
      robot_.updateBodyPose(imu_state_->getRPY(), imu_state_->getGyroscope(), imu_state_->getAccelerometer());
      /*update joint state*/
      robot_.updateAllJointState(*joints_state_);
      /*updata remote control data*/
      
      /*forward kinematics*/
      std::array<float, 2> rev_phi1, rev_phi4, rev_phi0, rev_L0;
      rev_phi1[0] = robot_.getHipJoints()[0].q;		// joint space map to fivelink joint frame
      rev_phi4[0] = robot_.getHipJoints()[1].q;
      rev_phi1[1] = robot_.getHipJoints()[2].q;
      rev_phi4[1] = robot_.getHipJoints()[3].q;
      robot_.forwardKinematics(rev_phi1, rev_phi4, rev_phi0, rev_L0, true);

      /*forward dynamics*/
      std::array<float, 2> rev_torqueA, rev_torqueE, rev_Tp, rev_F;
      rev_torqueA[0] = robot_.getHipJoints()[0].tau;
      rev_torqueE[0] = robot_.getHipJoints()[1].tau;
      rev_torqueA[1] = robot_.getHipJoints()[2].tau;
      rev_torqueE[1] = robot_.getHipJoints()[3].tau;
      robot_.forwardDynamics(rev_torqueA, rev_torqueE, rev_Tp, rev_F, true);

      /**/
      std::array<float, 2> rev_alpha, rev_theta;
      rev_alpha[0] = robot_.getVMCJointFrames()[0].alpha;
      rev_alpha[1] = robot_.getVMCJointFrames()[1].alpha;
      rev_theta[0] = robot_.getVMCJointFrames()[0].theta;
      rev_theta[1] = robot_.getVMCJointFrames()[1].theta;   
      state_estimator_.update(rev_alpha, rev_theta, rev_L0);

      /*XV Estimate*/
      std::array<float, 2> w_ecd;
      w_ecd[0] = robot_.getWheelJoints()[0].dq;
      w_ecd[1] = robot_.getWheelJoints()[1].dq;;
      state_estimator_.estimateVelocity(
        w_ecd, 
        robot_.getBodyWorldFrame().a[0],
        robot_.getBodyWorldFrame().rpy[1],
        robot_.getConfig().wheel_link[0].lengthOrRadius
      );

      /*Fn Estimate*/
      state_estimator_.estimateForce(
        rev_F,
        rev_Tp,
        robot_.getBodyWorldFrame().a[2] - 9.8,
        robot_.getConfig().wheel_link[0].mass
      );

      /*lqr Control*/
      LQRController6x2::StateVector lqr_xd;
      std::array<LQRController6x2::StateVector, 2> lqr_x;
      for(size_t index = 0; index < 2; index++){
        lqr_x[index](0) = robot_.getVMCJointFrames()[index].theta; 
        lqr_x[index](1) = (
           robot_.getVMCJointFrames()[index].theta_history[0] 
          -robot_.getVMCJointFrames()[index].theta_history[1]
        )/period_sec; 
        lqr_x[index](2) = state_estimator_.getVelocityEstimate().x_filter;
        lqr_x[index](3) = state_estimator_.getVelocityEstimate().v_filter;
        lqr_x[index](4) = -robot_.getBodyWorldFrame().rpy[1];
        lqr_x[index](5) = -robot_.getBodyWorldFrame().w[1];
        /*load lqr k table*/
        lqr_controller_.updateKFormLegLength(index, rev_L0[index]);
        if(0 && state_estimator_.detectGround()){
          // lqr_x[index](0) = 0;  //chassis_ctrl_.target_x[2];
          // lqr_x[index](1) = 0;  //chassis_ctrl_.target_x[3];
          lqr_x[index](2) = 0;  //chassis_ctrl_.target_x[2];
          lqr_x[index](3) = 0;  //chassis_ctrl_.target_x[3];
          state_estimator_.corverVelocityEstimate(0, 0);
          
          // lqr_controller_.setGainMatrix(index, 0, 0, 0);
          // lqr_controller_.setGainMatrix(index, 0, 1, 0);
          lqr_controller_.setGainMatrix(index, 0, 2, 0);
          lqr_controller_.setGainMatrix(index, 0, 3, 0);
          // lqr_controller_.setGainMatrix(index, 0, 4, 0);
          // lqr_controller_.setGainMatrix(index, 0, 5, 0);
          // lqr_controller_.setGainMatrix(index, 1, 0, 0);
          // lqr_controller_.setGainMatrix(index, 1, 1, 0);
          lqr_controller_.setGainMatrix(index, 1, 2, 0);
          lqr_controller_.setGainMatrix(index, 1, 3, 0);
          // lqr_controller_.setGainMatrix(index, 1, 4, 0);
          // lqr_controller_.setGainMatrix(index, 1, 5, 0);
        }else{

        }
        
      }
      auto lqr_u = lqr_controller_.calculate(lqr_xd, lqr_x);

      /*leg control*/
      auto leg_u = leg_controller_.calculate(
        target_L0, 
        rev_L0, 
        0, 
        robot_.getBodyWorldFrame().rpy[0], 
        rev_theta, 
        rev_phi0,
        robot_.getConfig().body_link.mass*G
      );

      /*body yaw rotate control*/
      std::array<float, 2> rotate_t = {0};  		// [PID]
      // rotate_t[0] = rotate_controller_.calculate(0, robot_.getBodyWorldFrame().w[2]);
      // rotate_t[1] = -rotate_t[0];

      /*inverse dynamics*/
      std::array<float, 2> set_Tp, set_F, set_T, set_torqueA, set_torqueE;
      for(int i = 0; i < 2 ;i++)
      {

        set_Tp[i] = lqr_u[i](1) + leg_u.compensation_tp[i];
        set_F[i]	= leg_u.F[i];
        set_T[i] = lqr_u[i](0) + rotate_t[i];
      }
      robot_.inverseDynamics(set_Tp, set_F, set_torqueA, set_torqueE);


      /*PT-Mix Control*/
      // limit the q-v-t value
      for(int i = 0; i < 2; i++){
        if(set_torqueA[i] > 50.0f){
          set_torqueA[i] = 50.0f;
        }else if(set_torqueA[i] <-50.0f){
          set_torqueA[i] = -50.0f;
        }
        if(set_torqueE[i] > 50.0f){
          set_torqueE[i] = 50.0f;
        }else if(set_torqueE[i] <-50.0f){
          set_torqueE[i] = -50.0f;
        }

        if(set_T[i] > 5.0f){
          set_T[i] = 5.0f;
        }else if(set_T[i] <-5.0f){
          set_T[i] = -5.0f;
        }
      }


      std::array<float, 6> set_joints_p = {0};
      std::array<float, 6> set_joints_v = {0};
      std::array<float, 6> set_joints_t = {
        set_torqueA[0],
        set_torqueE[0],
        set_T[0],
        set_torqueA[1],
        set_torqueE[1],
        set_T[1],
      };
      for(int i = 0; i < 6; i++){

        set_joints_p[i] = (robot_.getConfig().joints[i].invert_pos ? -1 : 1) * (set_joints_p[i] - robot_.getConfig().joints[i].pos_offset);
        set_joints_v[i] = (robot_.getConfig().joints[i].invert_vel ? -1 : 1) * set_joints_v[i];
        set_joints_t[i] = (robot_.getConfig().joints[i].invert_torque ? -1 : 1) * set_joints_t[i];
        
        joints_cmd_->setMode(i, JointCmdInterface::MODE_MIT);
        joints_cmd_->setPosition(i, set_joints_p[i]);
        joints_cmd_->setVelocity(i, set_joints_v[i]);
        joints_cmd_->setEffort(i, set_joints_t[i]);
        joints_cmd_->setKp(i, 0);
        joints_cmd_->setKd(i, 0);
      }

      std::chrono::duration<double> period(period_sec);
      std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::microseconds>(period));
    }
  }

  /*参数管理对象*/
  std::unique_ptr<ChassisConfigManager> config_manager_;
  std::atomic<bool> params_changed_{false};
  std::mutex param_mutex_;

  std::unique_ptr<IMUStateInterface> imu_state_;
  std::unique_ptr<JointStateInterface> joints_state_;
  std::unique_ptr<JointCmdInterface> joints_cmd_;
  std::unique_ptr<ChassisCtrlInterface> chassis_ctrl_;
  std::unique_ptr<ChassisStateInterface> chassis_state_;

  rclcpp::Subscription<wheel_legged_msgs::msg::IMUState>::SharedPtr imu_state_sub_;
  rclcpp::Subscription<wheel_legged_msgs::msg::JointStates>::SharedPtr joints_state_sub_;
  rclcpp::Publisher<wheel_legged_msgs::msg::JointCmds>::SharedPtr joints_cmd_pub_;
  rclcpp::Subscription<wheel_legged_msgs::msg::ChassisCtrl>::SharedPtr chassis_ctrl_sub_;
  rclcpp::Publisher<wheel_legged_msgs::msg::ChassisState>::SharedPtr chassis_state_pub_;
  rclcpp::TimerBase::SharedPtr joint_cmd_pubTimer_;
  rclcpp::TimerBase::SharedPtr chassis_state_pubTimer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;

  /*joint 有限状态机*/
  FSMEvent event_;
  std::unique_ptr<JFSM> jfsm_;    
  rclcpp::TimerBase::SharedPtr jfsm_timer_;

  /*模型对象*/
  WheelLeggedRobot robot_;
  /*控制器对象*/
  LQRController6x2 lqr_controller_;
  LegController leg_controller_;
  RotateController rotate_controller_;
  StateEstimator state_estimator_;
  /*控制线程对象*/
  std::thread control_thread_;
};

int main(int argc, char** argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<ChassisControlNode>("chassis_control_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}