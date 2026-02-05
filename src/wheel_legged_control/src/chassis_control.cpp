#include "rclcpp/rclcpp.hpp"
#include "Eigen/Dense"
#include <cstdint>
#include <iostream>
#include <memory>
#include <ostream>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/utilities.hpp>
#include <wheel_legged_msgs/msg/detail/chassis_ctrl__struct.hpp>
#include <wheel_legged_msgs/msg/detail/chassis_joint_cmd__struct.hpp>
#include <wheel_legged_msgs/msg/detail/chassis_joint_state__struct.hpp>
#include <wheel_legged_msgs/msg/detail/chassis_state__struct.hpp>
#include "Application/legControl.hpp"
#include "Application/rotateControl.hpp"
#include "Application/stateEstimate.hpp"
#include "Controllers/lqr.hpp"
#include "Engine/engine.hpp"
#include "wheel_legged_msgs/msg/imu_state.hpp"
#include "wheel_legged_msgs/msg/chassis_ctrl.hpp"
#include "wheel_legged_msgs/msg/chassis_state.hpp"
#include "wheel_legged_msgs/msg/chassis_joint_cmd.hpp"
#include "wheel_legged_msgs/msg/chassis_joint_state.hpp"

#define CHASSIS_HIPJOINT_MIN_POS -PI/36
#define CHASSIS_HIPJOINT_MAX_POS PI*4/9

namespace chassis_control{
  class Ctrl{
  public:
    Ctrl(){}
    float target_x[6] = {0};    
    float target_L0[2] = {0};   
    float target_Roll = {0};
    float target_wYaw = {0};    
    float target_fs[2]={0}; 
  };

  class IMUState{
  public:
    IMUState(){}
    float quaternion[4];
    float gyr[3];
    float acc[3];
    float rpy[3];
  };
  class JointState{
  public:
    JointState() : mode(0), q(0), dq(0), tau(0), tau_est(0){}
    uint8_t mode;
    float q;
    float dq;
    float tau;
    float tau_est;
  };
  class JointCmd{
  public:
    JointCmd() : mode(0), q(0), dq(0), tau(0),
                 p_kp(0), p_kd(0), v_kp(0),v_kd(0){}
    uint8_t mode;
    float q;
    float dq;
    float tau;
    float p_kp;
    float p_kd;
    float v_kp;
    float v_kd;
  };

  class State{
  public:
    State(){}
    IMUState imu;
    JointState joint[6];
  };
  class Cmd{
  public:
    Cmd(){}
    JointCmd joint[6];
  };
} //namespace chassis control 


class ChassisControlNode : public rclcpp::Node{
public:
  ChassisControlNode(std::string name):Node(name){
    RCLCPP_INFO(this->get_logger(), "%s节点已启动.", name.c_str());

    // 创建 订阅者  
    chassis_imuState_subscriber_ = this->create_subscription<wheel_legged_msgs::msg::IMUState>(
      "simulation/IMUState", 10, 
      std::bind(&ChassisControlNode::ChassisIMUState_callback, this, std::placeholders::_1));
    chassis_ctrl_subscriber_ = this->create_subscription<wheel_legged_msgs::msg::ChassisCtrl>(
      "sport/chassisCtrl", 10, 
      std::bind(&ChassisControlNode::ChassisCtrl_callback, this, std::placeholders::_1));     
    chassis_jointState_subscriber_ = this->create_subscription<wheel_legged_msgs::msg::ChassisJointState>(
        "simulation/chassisMotorState", 10, 
        std::bind(&ChassisControlNode::ChassisJointState_callback, this, std::placeholders::_1));     
    // 创建 发布者
    chassis_jointCmd_publisher_ = this->create_publisher<wheel_legged_msgs::msg::ChassisJointCmd>(
      "simulation/chassisMotorCmd", 10);
    chassis_state_publisher_ = this->create_publisher<wheel_legged_msgs::msg::ChassisState>(
      "sport/chassisState", 10);
    // 创建 定时器
    chassis_jointCmd_pubTimer_ = this->create_wall_timer(//发布 all motor state
      std::chrono::milliseconds(1),  // 1000Hz
      std::bind(&ChassisControlNode::JointCmd_timCallback, this));
    chassis_state_pubTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),  // 1000Hz
      std::bind(&ChassisControlNode::ChassisState_timCallback, this));
    // debug_timer_ = this->create_wall_timer(
    //   std::chrono::milliseconds(100), // 10Hz
    //   std::bind(&ChassisControlNode::Debug_PrintEngineModel, this));

    // 创建任务线程
    chassis_task_thread_ = std::thread(&ChassisControlNode::Chassis_Task, this);   
  }
  ~ChassisControlNode() {
    if (chassis_task_thread_.joinable()) {
      chassis_task_thread_.join();
    }
  }
private:

  void ChassisIMUState_callback(const wheel_legged_msgs::msg::IMUState::SharedPtr msg){
    for(int i = 0; i < 4; i++){
      chassis_state_.imu.quaternion[i] = msg->quaternion[i];
      if (i < 3){
        chassis_state_.imu.gyr[i] = msg->gyroscope[i];
        chassis_state_.imu.acc[i] = msg->accelerometer[i];
        chassis_state_.imu.rpy[i] = msg->rpy[i];
      }
    }
  }

  void ChassisJointState_callback(const wheel_legged_msgs::msg::ChassisJointState::SharedPtr msg){
    for(int i = 0; i < 6; i++){
      chassis_state_.joint[i].q = msg->joint_state[i].q;
      chassis_state_.joint[i].dq = msg->joint_state[i].dq;
      chassis_state_.joint[i].tau = msg->joint_state[i].tau;
      chassis_state_.joint[i].tau_est = msg->joint_state[i].tau_est;
    }
  }  
  
  void ChassisCtrl_callback(const wheel_legged_msgs::msg::ChassisCtrl::SharedPtr msg){
    for(int i = 0; i<6; i++){
        chassis_ctrl_.target_x[i]=msg->target_x[i];
    }
    for(int i = 0; i<2; i++){
      chassis_ctrl_.target_L0[i] = msg->target_l0[i];
      chassis_ctrl_.target_fs[i] = msg->target_fs[i];    
    }
    chassis_ctrl_.target_Roll = msg->target_roll;
    chassis_ctrl_.target_wYaw = msg->target_wyaw;
  }

  void JointCmd_timCallback(){
    auto msg= wheel_legged_msgs::msg::ChassisJointCmd();
    for(int i = 0; i < 6; i++){
      msg.joint_cmd[i].mode = chassis_cmd_.joint[i].mode;
      msg.joint_cmd[i].q = chassis_cmd_.joint[i].q;
      msg.joint_cmd[i].dq = chassis_cmd_.joint[i].dq;
      msg.joint_cmd[i].tau = chassis_cmd_.joint[i].tau;
      msg.joint_cmd[i].p_kp = chassis_cmd_.joint[i].p_kp;
      msg.joint_cmd[i].p_kd = chassis_cmd_.joint[i].p_kd;
      msg.joint_cmd[i].v_kp = chassis_cmd_.joint[i].v_kp;
      msg.joint_cmd[i].v_kd = chassis_cmd_.joint[i].v_kd;        
    }
    chassis_jointCmd_publisher_->publish(msg);
  }
  void ChassisState_timCallback(){
    auto msg = wheel_legged_msgs::msg::ChassisState();
    msg.header.frame_id = "chassis_state";
    msg.header.stamp = this->now();
    for(int i = 0; i < 3; i++){
      msg.body_worldframe.rpy[i] = rmInfantry_WLR.frame.body_worldFrame.rpy[i];
      msg.body_worldframe.gyro[i] = rmInfantry_WLR.frame.body_worldFrame.w[i];
      msg.body_worldframe.acc[i] = rmInfantry_WLR.frame.body_worldFrame.a[i];
    }
    for(int i = 0; i < 2; i++){
      msg.fivelink_forceframe[i].fx = rmInfantry_WLR.frame.fiveLink_forceFrame[i].Fx;
      msg.fivelink_forceframe[i].fy = rmInfantry_WLR.frame.fiveLink_forceFrame[i].Fy;

      msg.fivelink_jointframe[i].l0 = rmInfantry_WLR.frame.fiveLink_jointFrame[i].L0;
      for(int j = 0; j < 5; j++){
        msg.fivelink_jointframe[i].phi[j] = rmInfantry_WLR.frame.fiveLink_jointFrame[i].phi[j];
      }
      msg.legvmc_forceframe[i].f = rmInfantry_WLR.frame.vmc_forceFrame[i].F;
      msg.legvmc_forceframe[i].tp = rmInfantry_WLR.frame.vmc_forceFrame[i].Tp;
      msg.legvmc_jointframe[i].theta = rmInfantry_WLR.frame.vmc_jointFrame[i].theta;
      msg.legvmc_jointframe[i].aphi = rmInfantry_WLR.frame.vmc_jointFrame[i].aphi;
    }

    for(int i = 0; i < 2; i++){
      for(int j = 0; j < 6; j++){
        msg.lqrctrl[i].x[j] = lqrControllerHandle[i].X.matrixHandle(j);
        msg.lqrctrl[i].xd[j] = lqrControllerHandle[i].Xd.matrixHandle(j);
      }
      msg.lqrctrl[i].out[0] = lqrControllerHandle[i].Out.matrixHandle(0);
      msg.lqrctrl[i].out[1] = lqrControllerHandle[i].Out.matrixHandle(1);
    }

    for(int i = 0; i < 2; i++){
      msg.legctrl.spring_damping[i].set = leg_ControllerHandle.base[i].springDamping.set;
      msg.legctrl.spring_damping[i].fdb = leg_ControllerHandle.base[i].springDamping.fdb;
      msg.legctrl.spring_damping[i].out = leg_ControllerHandle.base[i].springDamping.out;
      msg.legctrl.feedforward_gravity[i] = leg_ControllerHandle.base[i].feedforward_gravity;
      msg.legctrl.feedforward_fs[i] = leg_ControllerHandle.base[i].feedforward_fs;
      msg.legctrl.f[i] = leg_ControllerHandle.base[i].F;
      msg.legctrl.compensation_tp[i] = leg_ControllerHandle.base[i].compensation_tp;
    }

    msg.legctrl.compensation_roll.set = leg_ControllerHandle.compensationRoll.set;
    msg.legctrl.compensation_roll.fdb = leg_ControllerHandle.compensationRoll.fdb;
    msg.legctrl.compensation_roll.out = leg_ControllerHandle.compensationRoll.out;
    msg.legctrl.compensation_phi0.set = leg_ControllerHandle.compensationPhi0.set;
    msg.legctrl.compensation_phi0.fdb = leg_ControllerHandle.compensationPhi0.fdb;
    msg.legctrl.compensation_phi0.out = leg_ControllerHandle.compensationPhi0.out;

    msg.rotatectrl.set = rotateControllerHandle.set;
    msg.rotatectrl.fdb = rotateControllerHandle.fdb;
    msg.rotatectrl.out = rotateControllerHandle.out;
    
    for(int i = 0; i < 2; i ++){
      msg.state_est.fn_est.ddot_zw[i] = stateEstimatorHandle.FnEst.ddot_zw[i];
      msg.state_est.fn_est.fn[i] = stateEstimatorHandle.FnEst.Fn[i];
      msg.state_est.xv_est.dot_xw[i] = stateEstimatorHandle.xvEst.dot_xw[i];
    }
    msg.state_est.xv_est.aver_vel = stateEstimatorHandle.xvEst.aver_vel;
    msg.state_est.xv_est.x_filter = stateEstimatorHandle.xvEst.x_filter;
    msg.state_est.xv_est.v_filter = stateEstimatorHandle.xvEst.v_filter;

    msg.ground_signal = ground_signal;
   
    chassis_state_publisher_->publish(msg);
  }

  void Chassis_Task(){

	  /*初始化模型*/
	  Engine_ModelInit(&rmInfantry_WLR);
	  /*初始化控制器*/
	  LQRControllerInit(&lqrControllerHandle[0]);
	  LQRControllerInit(&lqrControllerHandle[1]);
	  legControllerInit(&leg_ControllerHandle);
	  rotateControllerInit(&rotateControllerHandle);
    stateEstimatorInit(&stateEstimatorHandle, 0.003);
  
	  // SlipDetection_Init(&slipDetection_handle, 0.2, 0.05, 0.1);
    /*任务控制参数设置*/
    const uint8_t steptime = 3; // 3ms运行一次

	  chassis_ctrl_.target_L0[0] = 0.2;
	  chassis_ctrl_.target_L0[1] = 0.2;
    
    float xdot[2][6*1]= {{0}};
    float hipJoint_setP[4]={0};
    float hipJoint_setV[4]={0};
    float hipJoint_setT[4]={0};
    float wheelJoint_setT[2]={0};

	  while(rclcpp::ok())
	  {
		  /*update body euler angle*/
		  rmInfantry_WLR.frame.body_worldFrame.rpy[0] =    -chassis_state_.imu.rpy[0];
		  rmInfantry_WLR.frame.body_worldFrame.rpy[1] =    -chassis_state_.imu.rpy[1];
		  rmInfantry_WLR.frame.body_worldFrame.rpy[2] =     chassis_state_.imu.rpy[2];
		  /*update body gyro data*/
		  rmInfantry_WLR.frame.body_worldFrame.w[0] =  -chassis_state_.imu.gyr[0];
		  rmInfantry_WLR.frame.body_worldFrame.w[1] =  -chassis_state_.imu.gyr[1];
		  rmInfantry_WLR.frame.body_worldFrame.w[2] =   chassis_state_.imu.gyr[2];
      rmInfantry_WLR.frame.body_worldFrame.a[0] =  -chassis_state_.imu.acc[0];
		  rmInfantry_WLR.frame.body_worldFrame.a[1] =  -chassis_state_.imu.acc[1];
		  rmInfantry_WLR.frame.body_worldFrame.a[2] =   chassis_state_.imu.acc[2] - 9.8;
      /*update joint data*/
		  for(int i = 0; i < 4; i++){
			  // map to the kinmatics equation(fivelink frame)
        uint8_t index = hipJointConfig[i].index;
			  rmInfantry_WLR.joint.hipJoint[i].q = 
				  (hipJointConfig[i].ifInvertPos ? -1 : 1)*chassis_state_.joint[index].q + hipJointConfig[i].posOffset;
			  rmInfantry_WLR.joint.hipJoint[i].w = 
				  (hipJointConfig[i].ifInvertVel ? -1 : 1)*chassis_state_.joint[index].dq;
			  rmInfantry_WLR.joint.hipJoint[i].t = 
				  (hipJointConfig[i].ifInvertTorque ? -1 : 1)*chassis_state_.joint[index].tau;
		  }
		  /*update wheel joint data*/
		  for(int i = 0; i < 2; i++){
        uint8_t index = wheelJointConfig[i].index;
			  rmInfantry_WLR.joint.wheelJoint[i].q = 
				  (wheelJointConfig[i].ifInvertPos ? -1 : 1)*chassis_state_.joint[index].q;
			  rmInfantry_WLR.joint.wheelJoint[i].w = 
				  (wheelJointConfig[i].ifInvertVel ? -1 : 1)*chassis_state_.joint[index].dq;
			  rmInfantry_WLR.joint.wheelJoint[i].t = 
				  (wheelJointConfig[i].ifInvertTorque ? -1 : 1)*chassis_state_.joint[index].tau; //map to torque note!!!
		  }

		  /*updata remote control data*/
		  // target_x[3] = RC_USER.move_x;
      // chassis_ctrl_.target_x[2] += chassis_ctrl_.target_x[3]*(steptime/1000.0f);
      // chassis_ctrl_.target_x[3] = 0;
  
		  /*forward kinematics*/
		  float in_phi1[2], in_phi4[2], out_phi0[2], out_L0[2];
  
		  in_phi1[0] = rmInfantry_WLR.joint.hipJoint[0].q;		// joint space map to fivelink joint frame
		  in_phi4[0] = rmInfantry_WLR.joint.hipJoint[1].q;
		  in_phi1[1] = rmInfantry_WLR.joint.hipJoint[2].q;
		  in_phi4[1] = rmInfantry_WLR.joint.hipJoint[3].q;
		  Engine_ForwardKinematics(&rmInfantry_WLR, in_phi1, in_phi4, out_phi0, out_L0, 1);

      float alpha[2] , theta[2],  L0[2];
      alpha[0] = rmInfantry_WLR.frame.vmc_jointFrame[0].aphi;
      alpha[1] = rmInfantry_WLR.frame.vmc_jointFrame[1].aphi;
      theta[0] = rmInfantry_WLR.frame.vmc_jointFrame[0].aphi;
      theta[1] = rmInfantry_WLR.frame.vmc_jointFrame[1].aphi;
      L0[0] = rmInfantry_WLR.frame.vmc_jointFrame[0].aphi;
      L0[1] = rmInfantry_WLR.frame.vmc_jointFrame[1].aphi;      
      stateEstimatorUpdate(&stateEstimatorHandle, alpha, theta, L0);
      /*XV Estimate*/
      float w_ecd[2];
      w_ecd[0] = rmInfantry_WLR.joint.wheelJoint[0].w;
      w_ecd[1] = rmInfantry_WLR.joint.wheelJoint[1].w;
      xvEstimateCalc(&stateEstimatorHandle,     
        w_ecd,
        rmInfantry_WLR.frame.body_worldFrame.a[0],
        -rmInfantry_WLR.frame.body_worldFrame.rpy[1],
        rmInfantry_WLR.link.wheelLink[0].lengthOrRadius 
      );
      /*Fn Estimate*/
      float test_F[2] = {0};
      float test_Tp[2] = {0};
      test_F[0] = rmInfantry_WLR.frame.vmc_forceFrame[0].F;
      test_F[1] = rmInfantry_WLR.frame.vmc_forceFrame[1].F;
      test_Tp[0] = rmInfantry_WLR.frame.vmc_forceFrame[0].Tp;
      test_Tp[1] = rmInfantry_WLR.frame.vmc_forceFrame[1].Tp;
      FnEstimateCalc(&stateEstimatorHandle,
        test_F,
        test_Tp,
        rmInfantry_WLR.frame.body_worldFrame.a[2],
        rmInfantry_WLR.link.wheelLink->mass
      );      

		  /*lqr Control*/
		  float lqrOutput[2][2*1] = {{0}};
		  float lqrRevX[2][6*1] = {{0}};
		  float lqrOutput_T[2] = {0};
		  float lqrOutput_Tp[2] = {0};
		  for(int i = 0; i < 2; i++){
			  lqrRevX[i][0] =  rmInfantry_WLR.frame.vmc_jointFrame[i].theta;
			  lqrRevX[i][1] = (rmInfantry_WLR.frame.vmc_jointFrame[i].theta_arr[0] - rmInfantry_WLR.frame.vmc_jointFrame[i].theta_arr[1])*1000.0f/(steptime);
				lqrRevX[i][2] = stateEstimatorHandle.xvEst.x_filter;
			  lqrRevX[i][3] = (rmInfantry_WLR.joint.wheelJoint[0].w+rmInfantry_WLR.joint.wheelJoint[1].w)*rmInfantry_WLR.link.wheelLink[i].lengthOrRadius/2;
			  lqrRevX[i][4] = rmInfantry_WLR.frame.body_worldFrame.rpy[1];
			  lqrRevX[i][5] = rmInfantry_WLR.frame.body_worldFrame.w[1];
			  /*load lqr k table*/
        lqrControllerHandle[i].Gain.matrixHandle = Eigen::Map<Eigen::Matrix<float, 2, 6, Eigen::RowMajor>>(LQRKMatrixData);
			  /* */
        if(groundDetection(&stateEstimatorHandle)){
          ground_signal = 1;
          // if((stateEstimatorHandle.FnEst.Fn[0] < 50.0f && stateEstimatorHandle.FnEst.Fn[1] < 50.0f))
          std::cout << "ground !!!"<<std::endl;
          lqrRevX[i][2] = chassis_ctrl_.target_x[2];
          lqrRevX[i][3] = chassis_ctrl_.target_x[3];
          stateEstimatorHandle.xvEst.x_filter = chassis_ctrl_.target_x[2];
          // lqrControllerHandle[i].Gain.matrixHandle(0, 0) = 0;
          // lqrControllerHandle[i].Gain.matrixHandle(0, 1) = 0;
          lqrControllerHandle[i].Gain.matrixHandle(0, 2) = 0;
          lqrControllerHandle[i].Gain.matrixHandle(0, 3) = 0;
          // lqrControllerHandle[i].Gain.matrixHandle(0, 4) = 0;
          // lqrControllerHandle[i].Gain.matrixHandle(0, 5) = 0;
          // lqrControllerHandle[i].Gain.matrixHandle(1, 0) = 0;
          // lqrControllerHandle[i].Gain.matrixHandle(1, 1) = 0;
          lqrControllerHandle[i].Gain.matrixHandle(1, 2) = 0;
          lqrControllerHandle[i].Gain.matrixHandle(1, 3) = 0;
          // lqrControllerHandle[i].Gain.matrixHandle(1, 4) = 0;
          // lqrControllerHandle[i].Gain.matrixHandle(1, 5) = 0;
        }else{
          ground_signal = 0;
        }
			  LQRController_Calc(&lqrControllerHandle[i], chassis_ctrl_.target_x, &lqrRevX[i][0], &lqrOutput[i][0]);
			  lqrOutput_T[i] =  lqrOutput[i][0];
			  lqrOutput_Tp[i] = lqrOutput[i][1];
		  }
  
		  /*leg control*/
		  // float springDampingOut_F[2] = {0};		// [PID] vmc spring-damping output force 
		  // float feedforwardOut_F[2] = {0};			// [FeedForward] Fn + Fs
		  // float rollCompensation_F[2] = {0};		// [Compensation]
		  float phi0Compensation_Tp[2] = {0};			// [Compensation]
		  float rev_theta[2] = {0};
		  rev_theta[0] = rmInfantry_WLR.frame.vmc_jointFrame[0].theta;
		  rev_theta[1] = rmInfantry_WLR.frame.vmc_jointFrame[1].theta;
		  legControl(&leg_ControllerHandle, 
        chassis_ctrl_.target_L0, 
        out_L0, 
        chassis_ctrl_.target_Roll,
        rmInfantry_WLR.frame.body_worldFrame.rpy[0], 
        rev_theta, 
        chassis_ctrl_.target_fs, 
        out_phi0, 
        phi0Compensation_Tp);
  
		  /*body yaw rotate control*/
		  float yawRotateOut_T[2] = {0};  		// [PID]
		  yawRotateOut_T[1] = rotateControl(&rotateControllerHandle, chassis_ctrl_.target_wYaw, rmInfantry_WLR.frame.body_worldFrame.w[2]);
		  yawRotateOut_T[0] = -yawRotateOut_T[1];
  
		  /*inverse dynamics*/
		  float in_Tp[2], in_F[2], out_T[2];
      float out_torqueE[2], out_torqueA[2];
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
			  in_L0[i] = chassis_ctrl_.target_L0[i];
		  }
		  Engine_InverseKinematics(&rmInfantry_WLR, in_phi0, in_L0, out_phi1, out_phi4);


  
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

        if(out_torqueA[i] > 50.0f){
          out_torqueA[i] = 50.0f;
        }else if(out_torqueA[i] <-50.0f){
          out_torqueA[i] = -50.0f;
        }
        if(out_torqueE[i] > 50.0f){
          out_torqueE[i] = 50.0f;
        }else if(out_torqueE[i] <-50.0f){
          out_torqueE[i] = -50.0f;
        }

        if(wheelJoint_setT[i] > 5.0f){
          wheelJoint_setT[i] = 5.0f;
        }else if(wheelJoint_setT[i] <-5.0f){
          wheelJoint_setT[i] = -5.0f;
        }
		  }
      //fivelink joint frame map to joint space 
      for(int i = 0; i < 2; i++){
        hipJoint_setP[i*2 + 0] = out_phi1[i];
        hipJoint_setP[i*2 + 1] = out_phi4[i];
        hipJoint_setT[i*2 + 0] = out_torqueA[i];
        hipJoint_setT[i*2 + 1] = out_torqueE[i];
      }
		  for(int i=0; i < 4; i++){
			  hipJoint_setP[i] = (hipJointConfig[i].ifInvertPos ? -1 : 1) * (hipJoint_setP[i] - hipJointConfig[i].posOffset);
			  hipJoint_setV[i] = 0;
			  hipJoint_setT[i] = (hipJointConfig[i].ifInvertTorque ? -1 : 1) * hipJoint_setT[i];
        uint8_t index = hipJointConfig[i].index;
        chassis_cmd_.joint[index].mode = 0;
        chassis_cmd_.joint[index].q = hipJoint_setP[i];
        chassis_cmd_.joint[index].dq = hipJoint_setV[i];
        chassis_cmd_.joint[index].tau = hipJoint_setT[i];
        chassis_cmd_.joint[index].p_kp = 0;
        chassis_cmd_.joint[index].p_kd = 0;
        chassis_cmd_.joint[index].v_kp = 0;
        chassis_cmd_.joint[index].v_kd = 0;
		  }
		  for(int i = 0; i < 2; i++){
			  wheelJoint_setT[i] = (wheelJointConfig[i].ifInvertTorque ? -1 : 1) * out_T[i];

        uint8_t index = wheelJointConfig[i].index;
        chassis_cmd_.joint[index].mode = 0;
        chassis_cmd_.joint[index].q = 0;
        chassis_cmd_.joint[index].dq = 0;
        chassis_cmd_.joint[index].tau = wheelJoint_setT[i];
        chassis_cmd_.joint[index].p_kp = 0;
        chassis_cmd_.joint[index].p_kd = 0;
        chassis_cmd_.joint[index].v_kp = 0;
        chassis_cmd_.joint[index].v_kd = 0;
      }

    std::this_thread::sleep_for(std::chrono::milliseconds(steptime)); // 1kHz
    }  
  }
  
  chassis_control::Ctrl chassis_ctrl_;
  chassis_control::Cmd chassis_cmd_;
  chassis_control::State chassis_state_;

  LQRControllerHandle_t lqrControllerHandle[2];	//lqr controller
  LegControllerHandle_t leg_ControllerHandle;		//leg controller
  pid_type_def rotateControllerHandle;			    //rotate controller
  StateEstimatorHandle_t stateEstimatorHandle;
  // SlipDetectionHandle_t slipDetection_handle;

  std::thread chassis_task_thread_;
  rclcpp::Subscription<wheel_legged_msgs::msg::IMUState>::SharedPtr chassis_imuState_subscriber_;
  rclcpp::Subscription<wheel_legged_msgs::msg::ChassisCtrl>::SharedPtr chassis_ctrl_subscriber_;
  rclcpp::Publisher<wheel_legged_msgs::msg::ChassisState>::SharedPtr chassis_state_publisher_;
  rclcpp::Subscription<wheel_legged_msgs::msg::ChassisJointState>::SharedPtr chassis_jointState_subscriber_;
  rclcpp::Publisher<wheel_legged_msgs::msg::ChassisJointCmd>::SharedPtr chassis_jointCmd_publisher_;
  rclcpp::TimerBase::SharedPtr chassis_jointCmd_pubTimer_;
  rclcpp::TimerBase::SharedPtr chassis_state_pubTimer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;

  uint8_t ground_signal = 0;
};

int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<ChassisControlNode>("chassis_control_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}