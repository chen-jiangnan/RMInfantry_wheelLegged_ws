#include "rclcpp/rclcpp.hpp"
#include "Eigen/Dense"
#include <cstdint>
#include <iostream>
#include <memory>
#include <ostream>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <wheel_legged_msgs/msg/detail/chassis_joint_cmd__struct.hpp>
#include <wheel_legged_msgs/msg/detail/chassis_joint_state__struct.hpp>
#include "Application/legControl.hpp"
#include "Application/rotateControl.hpp"
#include "Controllers/lqr.hpp"
#include "Engine/engine.hpp"
#include "wheel_legged_msgs/msg/imu_state.hpp"
#include "wheel_legged_msgs/msg/chassis_cmd.hpp"
#include "wheel_legged_msgs/msg/chassis_joint_cmd.hpp"
#include "wheel_legged_msgs/msg/chassis_joint_state.hpp"

#define CHASSIS_HIPJOINT_MIN_POS -PI/36
#define CHASSIS_HIPJOINT_MAX_POS PI*4/9

class ChassisCmd{
public:
  ChassisCmd(){}
  float target_x[6] = {0};    
  float target_L0[2] = {0};   
  float target_Roll = {0};
  float target_wYaw = {0};    
  float target_fs[2]={0}; 
};
class ChassisIMUState{
public:
  ChassisIMUState(){}
  float quaternion[4];
  float gyr[3];
  float acc[3];
  float roll;
  float pitch;
  float yaw;
};
class ChassisJointState{
public:
  ChassisJointState() : mode(0), q(0), dq(0), tau(0), tau_est(0){}
    uint8_t mode;
    float q;
    float dq;
    float tau;
    float tau_est;
};
class ChassisState{
public:
  ChassisState(){}
  ChassisIMUState imu;
  ChassisJointState hipjoint[4];
  ChassisJointState wheeljoint[4];
};


class ChassisControlNode : public rclcpp::Node{
public:
  ChassisControlNode(std::string name):Node(name){
    RCLCPP_INFO(this->get_logger(), "%s节点已启动.", name.c_str());

    // 创建 订阅者  
    chassis_imuState_subscriber_ = this->create_subscription<wheel_legged_msgs::msg::IMUState>(
      "simulation/IMUState", 10, 
      std::bind(&ChassisControlNode::ChassisIMUState_callback, this, std::placeholders::_1));
    chassis_cmd_subscriber_ = this->create_subscription<wheel_legged_msgs::msg::ChassisCmd>(
      "sport/ChassisCmd", 10, 
      std::bind(&ChassisControlNode::ChassisCmd_callback, this, std::placeholders::_1));     
    chassis_jointState_subscriber_ = this->create_subscription<wheel_legged_msgs::msg::ChassisJointState>(
        "simulation/chassisMotorState", 10, 
        std::bind(&ChassisControlNode::ChassisJointState_callback, this, std::placeholders::_1));     
    // 创建 发布者
    chassis_jointCmd_publisher_ = this->create_publisher<wheel_legged_msgs::msg::ChassisJointCmd>(
      "simulation/chassisMotorCmd", 10);
    // 创建定时器发布 all motor state
    chassis_jointCmd_pubTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),  // 1000Hz
      std::bind(&ChassisControlNode::JointCmd_timCallback, this));
    debug_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 1000Hz
      std::bind(&ChassisControlNode::Debug_PrintEngineModel, this));

    // 创建任务线程
    chassis_task_thread_ = std::thread(&ChassisControlNode::Chassis_Task, this);   
  }
  ~ChassisControlNode() {
    if (chassis_task_thread_.joinable()) {
      chassis_task_thread_.join();
    }
  }
private:
void Debug_PrintEngineModel() {
  std::cout << "\n========== EngineModel Debug Info ==========" << std::endl;
  
  // 打印身体姿态
  std::cout << "\n--- Body World Frame ---" << std::endl;
  std::cout << "Roll:  " << std::setw(10) << std::fixed << std::setprecision(4) << rmInfantry_WLR.frame.body_worldFrame.roll 
            << " rad, Pitch: " << std::setw(10) << rmInfantry_WLR.frame.body_worldFrame.pitch
            << " rad, Yaw: " << std::setw(10) << rmInfantry_WLR.frame.body_worldFrame.yaw << " rad" << std::endl;
  std::cout << "Angular Vel: [" 
            << std::setw(8) << rmInfantry_WLR.frame.body_worldFrame.w[0] << ", "
            << std::setw(8) << rmInfantry_WLR.frame.body_worldFrame.w[1] << ", "
            << std::setw(8) << rmInfantry_WLR.frame.body_worldFrame.w[2] << "] rad/s" << std::endl;
  std::cout << "Acceleration: [" 
            << std::setw(8) << rmInfantry_WLR.frame.body_worldFrame.a[0] << ", "
            << std::setw(8) << rmInfantry_WLR.frame.body_worldFrame.a[1] << ", "
            << std::setw(8) << rmInfantry_WLR.frame.body_worldFrame.a[2] << "] m/s²" << std::endl;
  
  // 打印关节状态（按左右侧）
  for (int side = 0; side < 2; side++) {
      std::cout << "\n--- " << (side == 0 ? "Left Side" : "Right Side") << " ---" << std::endl;
      
      // 5连杆关节帧
      std::cout << "FiveLink Frame:" << std::endl;
      std::cout << "  L0:  " << std::setw(10) << rmInfantry_WLR.frame.fiveLink_jointFrame[side].L0 << " m" << std::endl;
      std::cout << "  Phi: [";
      for (int i = 0; i < 5; i++) {
          std::cout << std::setw(8) << rmInfantry_WLR.frame.fiveLink_jointFrame[side].phi[i];
          if (i < 4) std::cout << ", ";
      }
      std::cout << "] rad" << std::endl;
      
      // VMC关节帧
      std::cout << "VMC Joint Frame:" << std::endl;
      std::cout << "  Theta: " << std::setw(10) << rmInfantry_WLR.frame.vmc_jointFrame[side].theta 
                << " rad, APhi: " << std::setw(10) << rmInfantry_WLR.frame.vmc_jointFrame[side].aphi << " rad" << std::endl;
      
      // VMC力帧
      std::cout << "VMC Force Frame:" << std::endl;
      std::cout << "  Tp: " << std::setw(10) << rmInfantry_WLR.frame.vmc_forceFrame[side].Tp 
                << " N·m, F: " << std::setw(10) << rmInfantry_WLR.frame.vmc_forceFrame[side].F << " N" << std::endl;
      
      // 髋关节（4个） - 修改为左: 0,3, 右: 1,2
      std::cout << "Hip Joints:" << std::endl;
      if (side == 0) {  // 左: 0,3
          std::cout << "  Joint 0: q=" << std::setw(8) << rmInfantry_WLR.joint.hipJoint[0].q 
                    << " rad, w=" << std::setw(8) << rmInfantry_WLR.joint.hipJoint[0].w 
                    << " rad/s, t=" << std::setw(8) << rmInfantry_WLR.joint.hipJoint[0].t << " N·m" << std::endl;
          std::cout << "  Joint 3: q=" << std::setw(8) << rmInfantry_WLR.joint.hipJoint[3].q 
                    << " rad, w=" << std::setw(8) << rmInfantry_WLR.joint.hipJoint[3].w 
                    << " rad/s, t=" << std::setw(8) << rmInfantry_WLR.joint.hipJoint[3].t << " N·m" << std::endl;
      } else {  // 右: 1,2
          std::cout << "  Joint 1: q=" << std::setw(8) << rmInfantry_WLR.joint.hipJoint[1].q 
                    << " rad, w=" << std::setw(8) << rmInfantry_WLR.joint.hipJoint[1].w 
                    << " rad/s, t=" << std::setw(8) << rmInfantry_WLR.joint.hipJoint[1].t << " N·m" << std::endl;
          std::cout << "  Joint 2: q=" << std::setw(8) << rmInfantry_WLR.joint.hipJoint[2].q 
                    << " rad, w=" << std::setw(8) << rmInfantry_WLR.joint.hipJoint[2].w 
                    << " rad/s, t=" << std::setw(8) << rmInfantry_WLR.joint.hipJoint[2].t << " N·m" << std::endl;
      }
      
      // 轮关节
      std::cout << "Wheel Joint " << side << ":" << std::endl;
      std::cout << "  q=" << std::setw(8) << rmInfantry_WLR.joint.wheelJoint[side].q 
                << " rad, w=" << std::setw(8) << rmInfantry_WLR.joint.wheelJoint[side].w 
                << " rad/s, t=" << std::setw(8) << rmInfantry_WLR.joint.wheelJoint[side].t << " N·m" << std::endl;
  }
  
  std::cout << "============================================\n" << std::endl;

  std::ios_base::fmtflags old_flags = std::cout.flags();
  std::streamsize old_precision = std::cout.precision();
  
  std::cout << std::fixed << std::setprecision(4);
  
  for (int ctrl_idx = 0; ctrl_idx < 2; ctrl_idx++) {
      std::cout << "[" << (ctrl_idx == 0 ? "Left" : "Right") << " Leg LQR Controller]" << std::endl;
      
      const auto& lqr = lqrControllerHandle[ctrl_idx];
      
      // 打印 Gain 矩阵 (2x6)
      std::cout << "  Gain Matrix (2x6):" << std::endl;
      // for (int i = 0; i < 2; i++) {
      //     std::cout << "    Row " << i << ": [";
      //     for (int j = 0; j < 6; j++) {
      //         std::cout << std::setw(8) << lqr.Gain.matrixHandle.data()[i*6 + j];
      //         if (j < 5) std::cout << ", ";
      //     }
      //     std::cout << "]" << std::endl;
      // }
      std::cout<<lqr.Gain.matrixHandle << std::endl;
      
      // 打印状态 X (6x1)
      std::cout << "\n  State X (6x1):" << std::endl;
      std::cout << "    Theta:    " << std::setw(8) << lqr.X.matrixHandle.data()[0] << " rad" << std::endl;
      std::cout << "    Theta_dot:" << std::setw(8) << lqr.X.matrixHandle.data()[1] << " rad/s" << std::endl;
      std::cout << "    x:        " << std::setw(8) << lqr.X.matrixHandle.data()[2] << " m" << std::endl;
      std::cout << "    x_dot:    " << std::setw(8) << lqr.X.matrixHandle.data()[3] << " m/s" << std::endl;
      std::cout << "    Phi:      " << std::setw(8) << lqr.X.matrixHandle.data()[4] << " rad" << std::endl;
      std::cout << "    Phi_dot:  " << std::setw(8) << lqr.X.matrixHandle.data()[5] << " rad/s" << std::endl;
      
      // 打印期望状态 Xd (6x1)
      std::cout << "\n  Desired State Xd (6x1):" << std::endl;
      std::cout << "    Theta:    " << std::setw(8) << lqr.Xd.matrixHandle.data()[0] << " rad" << std::endl;
      std::cout << "    Theta_dot:" << std::setw(8) << lqr.Xd.matrixHandle.data()[1] << " rad/s" << std::endl;
      std::cout << "    x:        " << std::setw(8) << lqr.Xd.matrixHandle.data()[2] << " m" << std::endl;
      std::cout << "    x_dot:    " << std::setw(8) << lqr.Xd.matrixHandle.data()[3] << " m/s" << std::endl;
      std::cout << "    Phi:      " << std::setw(8) << lqr.Xd.matrixHandle.data()[4] << " rad" << std::endl;
      std::cout << "    Phi_dot:  " << std::setw(8) << lqr.Xd.matrixHandle.data()[5] << " rad/s" << std::endl;
      
      // 打印误差 Err (6x1)
      std::cout << "\n  Error (Xd - X) (6x1):" << std::endl;
      std::cout << "    dTheta:   " << std::setw(8) << lqr.Err.matrixHandle.data()[0] << " rad" << std::endl;
      std::cout << "    dTheta_dot:" << std::setw(7) << lqr.Err.matrixHandle.data()[1] << " rad/s" << std::endl;
      std::cout << "    dx:       " << std::setw(8) << lqr.Err.matrixHandle.data()[2] << " m" << std::endl;
      std::cout << "    dx_dot:   " << std::setw(8) << lqr.Err.matrixHandle.data()[3] << " m/s" << std::endl;
      std::cout << "    dPhi:     " << std::setw(8) << lqr.Err.matrixHandle.data()[4] << " rad" << std::endl;
      std::cout << "    dPhi_dot: " << std::setw(8) << lqr.Err.matrixHandle.data()[5] << " rad/s" << std::endl;
      
      // 打印输出 Out (2x1)
      std::cout << "\n  Control Output (2x1):" << std::endl;
      std::cout << "    Tp: " << std::setw(10) << lqr.Out.matrixHandle.data()[0] << " N·m (绕机体转轴力矩)" << std::endl;
      std::cout << "    T:  " << std::setw(10) << lqr.Out.matrixHandle.data()[1] << " N (沿杆长方向力)" << std::endl;
      
      if (ctrl_idx < 2 - 1) {
          std::cout << "\n" << std::string(50, '-') << "\n" << std::endl;
      }
  }
  
  std::cout << "\n============================================\n" << std::endl;
  
  // 恢复cout状态
  std::cout.flags(old_flags);
  std::cout.precision(old_precision);
  
}

  void ChassisIMUState_callback(const wheel_legged_msgs::msg::IMUState::SharedPtr msg){
    for(int i = 0; i<4; i++){
      chassis_state.imu.quaternion[i] = msg->quaternion[i];
    }
    for(int i = 0; i<3; i++){
      chassis_state.imu.gyr[i] = msg->gyroscope[i];
      chassis_state.imu.acc[i] = msg->accelerometer[i];
    }
    chassis_state.imu.roll = msg->rpy[0];
    chassis_state.imu.pitch = msg->rpy[1];
    chassis_state.imu.yaw = msg->rpy[2];    
  }
  void ChassisJointState_callback(const wheel_legged_msgs::msg::ChassisJointState::SharedPtr msg){
    chassis_state.hipjoint[0].q = msg->joint_state[0].q;
    chassis_state.hipjoint[0].dq = msg->joint_state[0].dq;
    chassis_state.hipjoint[0].tau = msg->joint_state[0].tau;
    chassis_state.hipjoint[0].tau_est = msg->joint_state[0].tau_est;
    chassis_state.hipjoint[1].q = msg->joint_state[3].q;
    chassis_state.hipjoint[1].dq = msg->joint_state[3].dq;
    chassis_state.hipjoint[1].tau = msg->joint_state[3].tau;
    chassis_state.hipjoint[1].tau_est = msg->joint_state[3].tau_est;
    chassis_state.wheeljoint[0].q = msg->joint_state[2].q;
    chassis_state.wheeljoint[0].dq = msg->joint_state[2].dq;
    chassis_state.wheeljoint[0].tau = msg->joint_state[2].tau;
    chassis_state.wheeljoint[0].tau_est = msg->joint_state[2].tau_est;

    chassis_state.hipjoint[2].q = msg->joint_state[4].q;
    chassis_state.hipjoint[2].dq = msg->joint_state[4].dq;
    chassis_state.hipjoint[2].tau = msg->joint_state[4].tau;
    chassis_state.hipjoint[2].tau = msg->joint_state[4].tau;
    chassis_state.hipjoint[3].q = msg->joint_state[1].q;
    chassis_state.hipjoint[3].dq = msg->joint_state[1].dq;
    chassis_state.hipjoint[3].tau = msg->joint_state[1].tau;
    chassis_state.hipjoint[3].tau_est = msg->joint_state[1].tau_est;
    chassis_state.wheeljoint[1].q = msg->joint_state[5].q;
    chassis_state.wheeljoint[1].dq = msg->joint_state[5].dq;
    chassis_state.wheeljoint[1].tau = msg->joint_state[5].tau;
    chassis_state.wheeljoint[1].tau_est = msg->joint_state[5].tau_est;
  }  
  void ChassisCmd_callback(const wheel_legged_msgs::msg::ChassisCmd::SharedPtr msg){
    for(int i = 0; i<6; i++){
      chassis_cmd.target_x[i]=msg->target_x[i];
    }
    for(int i = 0; i<2; i++){
      // chassis_cmd.target_L0[i]=msg->target_l0[i];
      chassis_cmd.target_L0[i] = 0.20;
      chassis_cmd.target_fs[i]=msg->target_fs[i];    
    }
    chassis_cmd.target_Roll=msg->target_roll;
    chassis_cmd.target_wYaw=msg->target_wyaw;
  }
  void JointCmd_timCallback(){
    auto msg= wheel_legged_msgs::msg::ChassisJointCmd();
    for(int i = 0; i < 6; i++){
      if(i==2 || i==5){
        msg.joint_cmd[i].p_kp = 0;
        msg.joint_cmd[i].p_kd = 0;      
      }
      else{
        msg.joint_cmd[i].p_kp = 10;
        msg.joint_cmd[i].p_kd = 1;
      }
    msg.joint_cmd[i].q = 0;
    msg.joint_cmd[i].dq = 0;
    msg.joint_cmd[i].tau = 0;
    }
    msg.joint_cmd[0].q = hipJoint_setP[0];
    msg.joint_cmd[0].dq = hipJoint_setV[0];
    msg.joint_cmd[0].tau = hipJoint_setT[0];
    msg.joint_cmd[1].q = hipJoint_setP[3];
    msg.joint_cmd[1].dq = hipJoint_setV[3];
    msg.joint_cmd[1].tau = hipJoint_setT[3];
    msg.joint_cmd[2].q =  0;
    msg.joint_cmd[2].dq = 0;
    msg.joint_cmd[2].tau = wheelJoint_setT[0];

    msg.joint_cmd[3].q = hipJoint_setP[1];
    msg.joint_cmd[3].dq = hipJoint_setV[1];
    msg.joint_cmd[3].tau = hipJoint_setT[1];
    msg.joint_cmd[4].q = hipJoint_setP[2];
    msg.joint_cmd[4].dq = hipJoint_setV[2];
    msg.joint_cmd[4].tau = hipJoint_setT[2];
    msg.joint_cmd[5].q =  0;
    msg.joint_cmd[5].dq = 0;
    msg.joint_cmd[5].tau = wheelJoint_setT[1];
    chassis_jointCmd_publisher_->publish(msg);
  }
  void Chassis_Task(){
	  /*初始化模型*/
	  Engine_ModelInit(&rmInfantry_WLR);
	  /*初始化控制器*/
	  LQRControllerInit(&lqrControllerHandle[0]);
	  LQRControllerInit(&lqrControllerHandle[1]);
	  legControllerInit(&leg_ControllerHandle);
	  rotateControllerInit(&rotateControllerHandle);
  
	  // SlipDetection_Init(&slipDetection_handle, 0.2, 0.05, 0.1);
	  chassis_cmd.target_L0[0] = 0.20;
	  chassis_cmd.target_L0[1] = 0.20;
	  while(rclcpp::ok())
	  {
		  /*update body euler angle*/
		  rmInfantry_WLR.frame.body_worldFrame.roll =    -chassis_state.imu.roll;
		  rmInfantry_WLR.frame.body_worldFrame.yaw =      chassis_state.imu.yaw ;
		  rmInfantry_WLR.frame.body_worldFrame.pitch =   -chassis_state.imu.pitch;
		  /*update body gyro data*/
		  rmInfantry_WLR.frame.body_worldFrame.w[0] = -chassis_state.imu.gyr[1];
		  rmInfantry_WLR.frame.body_worldFrame.w[2] =  chassis_state.imu.gyr[2];
		  rmInfantry_WLR.frame.body_worldFrame.w[1] = -chassis_state.imu.gyr[0];
		  /*update hip joint data*/
		  for(int i = 0; i < 4; i++){
			  // map to the kinmatics equation(fivelink frame) 
			  rmInfantry_WLR.joint.hipJoint[i].q = 
				  (hipJointConfig[i].ifInvertPos ? -1 : 1)*chassis_state.hipjoint[i].q + hipJointConfig[i].posOffset;
			  rmInfantry_WLR.joint.hipJoint[i].w = 
				  (hipJointConfig[i].ifInvertVel ? -1 : 1)*chassis_state.hipjoint[i].dq;
			  rmInfantry_WLR.joint.hipJoint[i].t = 
				  (hipJointConfig[i].ifInvertTorque ? -1 : 1)*chassis_state.hipjoint[i].tau;		
		  }
		  /*update wheel joint data*/
		  for(int i = 0; i < 2; i++){
			  rmInfantry_WLR.joint.wheelJoint[i].q = 
				  (wheelJointConfig[i].ifInvertPos ? -1 : 1)*chassis_state.wheeljoint[i].q;
			  rmInfantry_WLR.joint.wheelJoint[i].w = // dps/lsb map to m/s
				  (wheelJointConfig[i].ifInvertVel ? -1 : 1)*chassis_state.wheeljoint[i].dq * rmInfantry_WLR.link.wheelLink[i].lengthOrRadius;
			  rmInfantry_WLR.joint.wheelJoint[i].t = 
				  (wheelJointConfig[i].ifInvertTorque ? -1 : 1)*chassis_state.wheeljoint[i].tau; //map to torque note!!!
		  }
		  /*updata remote control data*/
		  // target_x[3] = RC_USER.move_x;
  
		  /*forward kinematics*/
		  float in_phi1[2], in_phi4[2], out_phi0[2], out_L0[2];
  
		  in_phi1[0] = rmInfantry_WLR.joint.hipJoint[0].q;		// joint space map to fivelink joint frame
		  in_phi4[0] = rmInfantry_WLR.joint.hipJoint[3].q;
		  in_phi1[1] = rmInfantry_WLR.joint.hipJoint[1].q;
		  in_phi4[1] = rmInfantry_WLR.joint.hipJoint[2].q;
		  Engine_ForwardKinematics(&rmInfantry_WLR, in_phi1, in_phi4, out_phi0, out_L0, 1);
  
		  /*lqr Control*/
		  float lqrOutput[2][2*1] = {{0}};
		  float lqrRevX[2][6*1] = {{0}};
		  float lqrOutput_T[2] = {0};
		  float lqrOutput_Tp[2] = {0};
		  for(int i = 0; i < 2; i++){
			  lqrRevX[i][0] =  rmInfantry_WLR.frame.vmc_jointFrame[i].theta;
			  lqrRevX[i][1] = (rmInfantry_WLR.frame.vmc_jointFrame[i].theta - rmInfantry_WLR.frame.vmc_jointFrame[i].last_theta)*1000.0f/(steptime);
				lqrRevX[i][2] = 0;
        // lqrRevX[i][3] = (rmInfantry_WLR.joint.wheelJoint[0].w + rmInfantry_WLR.joint.wheelJoint[1].w)/2;
			  lqrRevX[i][3] = rmInfantry_WLR.joint.wheelJoint[i].w;
			  lqrRevX[i][4] = rmInfantry_WLR.frame.body_worldFrame.pitch;
			  lqrRevX[i][5] = rmInfantry_WLR.frame.body_worldFrame.w[1];
			  /*load lqr k table*/
        lqrControllerHandle[i].Gain.matrixHandle = Eigen::Map<Eigen::Matrix<float, 2, 6, Eigen::RowMajor>>(LQRKMatrixData);
			  // memcpy(&lqrControllerHandle[i].Gain.matrixData, LQRKMatrixData, 6*2*sizeof(float));
			  /* */
			  if(chassis_cmd.target_x[3] != 0){ //如果目标速度不为零，则将关于位移的lqr权重置零
				  lqrControllerHandle[i].Gain.matrixHandle.data()[2] = 0; //T
				  lqrControllerHandle[i].Gain.matrixHandle.data()[8] = 0; //Tp
			  }
			  LQRController_Calc(&lqrControllerHandle[i], chassis_cmd.target_x, &lqrRevX[i][0], &lqrOutput[i][0]);
			  lqrOutput_T[i] =  lqrOutput[i][0];
			  lqrOutput_Tp[i] = lqrOutput[i][1];
		  }
  
		  float xn[2] = {lqrRevX[0][3], chassis_state.imu.acc[1]};
		  float zn[2] = {lqrRevX[0][3], chassis_state.imu.acc[1]};
		  // SlipDetection(&slipDetection_handle, xn, zn, 4.0/1000);
		  
		  /*leg control*/
		  // float springDampingOut_F[2] = {0};		// [PID] vmc spring-damping output force 
		  // float feedforwardOut_F[2] = {0};			// [FeedForward] Fn + Fs
		  // float rollCompensation_F[2] = {0};		// [Compensation]
		  float phi0Compensation_Tp[2] = {0};			// [Compensation]
		  float rev_theta[2] = {0};
		  rev_theta[0] = rmInfantry_WLR.frame.vmc_jointFrame[0].theta;
		  rev_theta[1] = rmInfantry_WLR.frame.vmc_jointFrame[1].theta;
		  legControl(&leg_ControllerHandle, 
        chassis_cmd.target_L0, 
        out_L0, 
        chassis_cmd.target_Roll,
        rmInfantry_WLR.frame.body_worldFrame.roll, 
        rev_theta, 
        chassis_cmd.target_fs, 
        out_phi0, 
        phi0Compensation_Tp);
  
		  /*body yaw rotate control*/
		  float yawRotateOut_T[2] = {0};  		// [PID]
		  yawRotateOut_T[1] = rotateControl(&rotateControllerHandle, chassis_cmd.target_wYaw, rmInfantry_WLR.frame.body_worldFrame.w[2]);
		  yawRotateOut_T[0] = -yawRotateOut_T[1];
  
		  /*inverse dynamics*/
		  float in_Tp[2], in_F[2], out_T[2];
		  for(int i = 0; i < 2 ;i++)
		  {
			  in_Tp[i] = lqrOutput_Tp[i] + phi0Compensation_Tp[i];
			  in_F[i]	= leg_ControllerHandle.base[i].F;
			  out_T[i] = lqrOutput_T[i] + yawRotateOut_T[i];
		  }
		  Engine_InverseDynamics(&rmInfantry_WLR, in_Tp, in_F, out_torqueA, out_torqueE, 1);

		  /*calculate system state-space X_dot  = A*X + B*U*/
		  float target_phi0[2] = {0};
		  float target_theta[2] = {0};
		  float target_phi[2] = {0};
		  for(int i = 0; i < 2; i++){
			  float u[2] = {0};
			  u[0] = out_T[i];
			  u[1] = in_Tp[i];
			  Engine_SystemSpaceCalculate(&lqrRevX[i][0], u, &xdot[i][0]);
			  // calculate velcotiy q_dot(t) = q_ddot(t-1)*steptime + q_dot(t-1)
			  // calculate position q(t) = q(t-1) + (q_ddot(t-1)*steptime + q_dot(t-1))*steptime
			  target_theta[i] = lqrRevX[i][0] + (xdot[i][0] + xdot[i][1]*steptime/1000)*steptime/1000;
			  target_phi[i] = lqrRevX[i][4] + (xdot[i][4] + xdot[i][5]*steptime/1000)*steptime/1000;
			  target_phi0[i] = target_theta[i] + target_phi[i] + PI/2;
      }

		  /*inverse kinmatics*/
		  float in_phi0[2], in_L0[2], out_phi1[2], out_phi4[2];
		  for(int i = 0; i <2; i++){
			  in_phi0[i] = target_phi0[i];
			  in_L0[i] = chassis_cmd.target_L0[i];
		  }
		  Engine_InverseKinematics(&rmInfantry_WLR, in_phi0, in_L0, NULL, out_phi1, out_phi4);
  
		  /*PT-Mix Control*/
		  // limit the q-v-t value
		  for(int i = 0; i < 2; i++){
			  if(out_phi1[i] < CHASSIS_HIPJOINT_MIN_POS){
				  out_phi1[i] = CHASSIS_HIPJOINT_MIN_POS;
			  }else if(out_phi1[i] > CHASSIS_HIPJOINT_MAX_POS){
				  out_phi1[i] = CHASSIS_HIPJOINT_MAX_POS;
			  }
			  if(out_phi4[i] < PI - CHASSIS_HIPJOINT_MAX_POS){
				  out_phi4[i] = PI - CHASSIS_HIPJOINT_MAX_POS;
			  }else if(out_phi4[i] > PI - CHASSIS_HIPJOINT_MIN_POS){
				  out_phi4[i] = PI - CHASSIS_HIPJOINT_MIN_POS;
			  }
		  }

		  for(int i=0; i < 4; i++){
			  hipJoint_setP[i] = (hipJointConfig[i].ifInvertPos ? -1 : 1) * ((i < 2 ? out_phi1[i] : out_phi4[3-i]) - hipJointConfig[i].posOffset);
			  hipJoint_setV[i] = 0;
			  hipJoint_setT[i] = (hipJointConfig[i].ifInvertTorque ? -1 : 1) * (i < 2 ? out_torqueA[i] : out_torqueE[3-i]);
		  }
		  for(int i = 0; i < 2; i++){
			  wheelJoint_setT[i] = (wheelJointConfig[i].ifInvertTorque ? -1 : 1) * out_T[i];
			  wheelTorque[i] = wheelJoint_setT[i];
      }
		//  vTaskDelay(steptime);
    std::this_thread::sleep_for(std::chrono::milliseconds(steptime)); // 1kHz
    }  
  }
  /*任务控制参数设置*/
  const uint8_t steptime = 4; // 4ms运行一次
  float out_torqueE[2], out_torqueA[2];
  float hipJoint_setP[4]={0};
  float hipJoint_setV[4]={0};
  float hipJoint_setT[4]={0};
  float wheelJoint_setT[2]={0};
  float wheelTorque[2] = {0};
  float xdot[2][6*1]= {{0}};
  ChassisCmd chassis_cmd;
  ChassisState chassis_state;
  LQRControllerHandle_t lqrControllerHandle[2];	//lqr controller
  LegControllerHandle_t leg_ControllerHandle;		//leg controller
  pid_type_def rotateControllerHandle;			    //rotate controller
  // SlipDetectionHandle_t slipDetection_handle;
  std::thread chassis_task_thread_;
  // std::atomic<bool> data_exchange_running_;
  rclcpp::Subscription<wheel_legged_msgs::msg::IMUState>::SharedPtr chassis_imuState_subscriber_;
  rclcpp::Subscription<wheel_legged_msgs::msg::ChassisCmd>::SharedPtr chassis_cmd_subscriber_;
  rclcpp::Subscription<wheel_legged_msgs::msg::ChassisJointState>::SharedPtr chassis_jointState_subscriber_;
  rclcpp::Publisher<wheel_legged_msgs::msg::ChassisJointCmd>::SharedPtr chassis_jointCmd_publisher_;
  rclcpp::TimerBase::SharedPtr chassis_jointCmd_pubTimer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;

};

int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<ChassisControlNode>("chassis_control_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}