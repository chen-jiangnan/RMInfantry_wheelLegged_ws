/**
  ****************************(C) COPYRIGHT 2025 征途****************************
  * @file       legControl.c/h
  * @brief      
  *             这里是5连杆腿长控制器，通过vmc将5连杆虚拟成弹簧阻尼器并且并联一个力源
  *             1.弹簧阻尼器控制
  *             2.机体roll角控制
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Feb-6-2025    chenjiangnan     1.
  *                                             
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 征途****************************
  */
  #include "legControl.hpp"

  #define BODY_GRIVITY 12.1846*9.8
  
  /** 
    * @brief          init legController handle
    * @param[in]      legcontroller: handle address
    * @retval         none
    */
  void legControllerInit(LegControllerHandle_t* legcontroller)
  {
      if(legcontroller == NULL ){
          return;
      }
  
      float configPID_SpringDamping[3]={
          LEGCONTROL_SPRINGDAMPING_PID_POSITION_KP,
          LEGCONTROL_SPRINGDAMPING_PID_POSITION_KI,
          LEGCONTROL_SPRINGDAMPING_PID_POSITION_KD
      };//位置环pid参数
  
      float configPID_CompensationRollT[3]={
          LEGCONTROL_COMPENSATIONROLL_PID_POSITION_KP,
          LEGCONTROL_COMPENSATIONROLL_PID_POSITION_KI,
          LEGCONTROL_COMPENSATIONROLL_PID_POSITION_KD
      }; //位置环pid参数
      
      float configPID_CompensationTheta0[3]={
          LEGCONTROL_COMPENSATIONPHI0_PID_POSITION_KP,
          LEGCONTROL_COMPENSATIONPHI0_PID_POSITION_KI,
          LEGCONTROL_COMPENSATIONPHI0_PID_POSITION_KD
      }; //位置环pid参数	
      for(int i = 0; i < 2; i++){
          PID_init(&legcontroller->base[i].springDamping, PID_POSITION, configPID_SpringDamping, 300, 300);
      }
      PID_init(&legcontroller->compensationRoll, PID_POSITION, configPID_CompensationRollT, 300, 300);
      PID_init(&legcontroller->compensationPhi0, PID_POSITION, configPID_CompensationTheta0, 10, 10);
  }
  
  
  /** 
    * @brief          pid control calculate
    * @param[in]      pid: pid controller handle address
    * @param[in]      ref: feedback value
    * @param[in]      set: target value
    * @retval         none
    */
  float SpringDampingPIDController(pid_type_def *pid, float ref, float set)
  {
      if (pid == NULL)
      {
          return 0.0f;
      }
  
      pid->error[2] = pid->error[1];
      pid->error[1] = pid->error[0];
      pid->set = set;
      pid->fdb = ref;
      pid->error[0] = set - ref;
      if (pid->mode == PID_POSITION)
      {
          pid->Pout = pid->Kp * pid->error[0];
          pid->Iout += pid->Ki * pid->error[0];
          pid->Dbuf[2] = pid->Dbuf[1];
          pid->Dbuf[1] = pid->Dbuf[0];
          pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
          pid->Dout = pid->Kd * pid->Dbuf[0];
          LimitMax(pid->Iout, pid->max_iout);
          pid->out = pid->Pout + pid->Iout + pid->Dout;
          LimitMax(pid->out, pid->max_out);
          
          return pid->out;
      }
      else if (pid->mode == PID_DELTA)
      {
          pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
          pid->Iout = pid->Ki * pid->error[0];
          pid->Dbuf[2] = pid->Dbuf[1];
          pid->Dbuf[1] = pid->Dbuf[0];
          pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
          pid->Dout = pid->Kd * pid->Dbuf[0];
          pid->out += pid->Pout + pid->Iout + pid->Dout;
          LimitMax(pid->out, pid->max_out);
          
          return pid->out;
      }
      else
      {
          return 0;
      }
  }
  
  
  /** 
    * @brief          pid control calculate
    * @param[in]      pid: pid controller handle address
    * @param[in]      ref: feedback value
    * @param[in]      set: target value
    * @retval         none
    */
  float CompensationRollPIDController(pid_type_def *pid, float ref, float set)
  {
      if (pid == NULL)
      {
          return 0.0f;
      }
  
      pid->error[2] = pid->error[1];
      pid->error[1] = pid->error[0];
      pid->set = set;
      pid->fdb = ref;
      pid->error[0] = set - ref;
      if (pid->mode == PID_POSITION)
      {
          pid->Pout = pid->Kp * pid->error[0];
          pid->Iout += pid->Ki * pid->error[0];
          pid->Dbuf[2] = pid->Dbuf[1];
          pid->Dbuf[1] = pid->Dbuf[0];
          pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
          pid->Dout = pid->Kd * pid->Dbuf[0];
          LimitMax(pid->Iout, pid->max_iout);
          pid->out = pid->Pout + pid->Iout + pid->Dout;
          LimitMax(pid->out, pid->max_out);
          
          return pid->out;
      }
      else if (pid->mode == PID_DELTA)
      {
          pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
          pid->Iout = pid->Ki * pid->error[0];
          pid->Dbuf[2] = pid->Dbuf[1];
          pid->Dbuf[1] = pid->Dbuf[0];
          pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
          pid->Dout = pid->Kd * pid->Dbuf[0];
          pid->out += pid->Pout + pid->Iout + pid->Dout;
          LimitMax(pid->out, pid->max_out);
          
          return pid->out;
      }
      else
      {
          return 0;
      }
  }
  
  /** 
    * @brief          pid control calculate
    * @param[in]      pid: pid controller handle address
    * @param[in]      ref: feedback value
    * @param[in]      set: target value
    * @retval         none
    */
  float CompensationPhi0PIDController(pid_type_def *pid, float ref, float set)
  {
      if (pid == NULL)
      {
          return 0.0f;
      }
  
      pid->error[2] = pid->error[1];
      pid->error[1] = pid->error[0];
      pid->set = set;
      pid->fdb = ref;
      pid->error[0] = set - ref;
      if (pid->mode == PID_POSITION)
      {
          pid->Pout = pid->Kp * pid->error[0];
          pid->Iout += pid->Ki * pid->error[0];
          pid->Dbuf[2] = pid->Dbuf[1];
          pid->Dbuf[1] = pid->Dbuf[0];
          pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
          pid->Dout = pid->Kd * pid->Dbuf[0];
          LimitMax(pid->Iout, pid->max_iout);
          pid->out = pid->Pout + pid->Iout + pid->Dout;
          LimitMax(pid->out, pid->max_out);
          
          return pid->out;
      }
      else if (pid->mode == PID_DELTA)
      {
          pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
          pid->Iout = pid->Ki * pid->error[0];
          pid->Dbuf[2] = pid->Dbuf[1];
          pid->Dbuf[1] = pid->Dbuf[0];
          pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
          pid->Dout = pid->Kd * pid->Dbuf[0];
          pid->out += pid->Pout + pid->Iout + pid->Dout;
          LimitMax(pid->out, pid->max_out);
          
          return pid->out;
      }
      else
      {
          return 0;
      }
  }
  
  
  /** 
    * @brief          leg control calculate
    * @param[in]      legcontroller: legController handle
    * @param[in]      target_L0: target L0 (m)
    * @param[in]      rev_L0: feedback L0  (m)
    * @param[in]      target_Roll: target Roll(Euler-Angle) (rad)
    * @param[in]      rev_Roll: feedback Roll(Euler-Angle) (rad)
    * @param[in]      theta: feedback theta (rad)
    * @param[in]      fs: target fs (N)
    * @retval         none
    */
  void legControl(
      LegControllerHandle_t* legcontroller, 
      const float* target_L0, 
      const float* rev_L0, 
      const float target_Roll,
      const float rev_Roll,
      const float* theta, 
      const float* fs,
      const float* phi0,
      float* compensation_tp
  )
  {
      if(legcontroller == NULL || target_L0 == NULL || rev_L0 == NULL || theta == NULL ){
          return;
      }
      if(phi0 == NULL || compensation_tp == NULL){
          return;
      }
      /*spring-damping controller paramter is not empty*/
      for(int i = 0; i < 2; i++){
          SpringDampingPIDController(&legcontroller->base[i].springDamping, rev_L0[i], target_L0[i]);
          legcontroller->base[i].feedforward_gravity = BODY_GRIVITY*std::cos(theta[i])/2;
          legcontroller->base[i].F = legcontroller->base[i].springDamping.out + legcontroller->base[i].feedforward_gravity;
      }
  
  //    if(target_Roll == NULL || rev_Roll == NULL){
  //        return;
  //    }
      /*compensation Roll controller paramter is not empty*/
      CompensationRollPIDController(&legcontroller->compensationRoll, rev_Roll, target_Roll);
      legcontroller->base[0].F += legcontroller->compensationRoll.out;
      legcontroller->base[1].F -= legcontroller->compensationRoll.out;
      
      float diff_theta = phi0[0]- phi0[1];
      CompensationPhi0PIDController(&legcontroller->compensationPhi0, diff_theta, 0);
      legcontroller->base[0].compensation_tp =  legcontroller->compensationPhi0.out;
      legcontroller->base[1].compensation_tp = -legcontroller->compensationPhi0.out;
      compensation_tp[0] = legcontroller->base[0].compensation_tp;
      compensation_tp[1] = legcontroller->base[1].compensation_tp;
      
  
      if(fs == NULL){
          legcontroller->base[0].feedforward_fs = 0;
          legcontroller->base[1].feedforward_fs = 0;
      }else{
          legcontroller->base[0].feedforward_fs = fs[0];
          legcontroller->base[1].feedforward_fs = fs[1];
      }
  
      legcontroller->base[0].F += legcontroller->base[0].feedforward_fs;
      legcontroller->base[1].F += legcontroller->base[1].feedforward_fs;  
  }