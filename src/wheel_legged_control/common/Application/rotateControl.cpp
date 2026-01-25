/**
  ****************************(C) COPYRIGHT 2025 征途****************************
  * @file       rotateControl.c/h
  * @brief      
  *             这里是机体yaw角旋转控制
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
#include "rotateControl.hpp"

/** 
  * @brief          init RotateController handle
  * @param[in]      rotateController: handle address
  * @retval         none
  */
void rotateControllerInit(pid_type_def* rotateController)
{
  float configPID_ROTATE[3] = {
    ROTATECONTROL_SPEEDPID_POSITION_KP,
    ROTATECONTROL_SPEEDPID_POSITION_KI,
    ROTATECONTROL_SPEEDPID_POSITION_KD
  };
  PID_init(rotateController, PID_POSITION, configPID_ROTATE, 3.0, 3.0);
}

/** 
  * @brief          iotateController calculate
  * @param[in]      rotateController:rotateControoler handle
  * @param[in]      target_wYaw:目标角速度
  * @param[in]      rev_wYaw:机体角速度反馈
  * @retval         none
  */
float rotateControl(pid_type_def* rotateController, float target_wYaw, float rev_wYaw)
{
  if (rotateController == NULL)
  {
      return 0.0f;
  }

  rotateController->error[2] = rotateController->error[1];
  rotateController->error[1] = rotateController->error[0];
  rotateController->set = target_wYaw;
  rotateController->fdb = rev_wYaw;
  rotateController->error[0] = target_wYaw - rev_wYaw;
  if (rotateController->mode == PID_POSITION)
  {
    rotateController->Pout = rotateController->Kp * rotateController->error[0];
    rotateController->Iout += rotateController->Ki * rotateController->error[0];
    rotateController->Dbuf[2] = rotateController->Dbuf[1];
    rotateController->Dbuf[1] = rotateController->Dbuf[0];
    rotateController->Dbuf[0] = (rotateController->error[0] - rotateController->error[1]);
    rotateController->Dout = rotateController->Kd * rotateController->Dbuf[0];
    LimitMax(rotateController->Iout, rotateController->max_iout);
    rotateController->out = rotateController->Pout + rotateController->Iout + rotateController->Dout;
    LimitMax(rotateController->out, rotateController->max_out);
  
  return rotateController->out;
  }
  else if (rotateController->mode == PID_DELTA)
  {
    rotateController->Pout = rotateController->Kp * (rotateController->error[0] - rotateController->error[1]);
    rotateController->Iout = rotateController->Ki * rotateController->error[0];
    rotateController->Dbuf[2] = rotateController->Dbuf[1];
    rotateController->Dbuf[1] = rotateController->Dbuf[0];
    rotateController->Dbuf[0] = (rotateController->error[0] - 2.0f * rotateController->error[1] + rotateController->error[2]);
    rotateController->Dout = rotateController->Kd * rotateController->Dbuf[0];
    rotateController->out += rotateController->Pout + rotateController->Iout + rotateController->Dout;
    LimitMax(rotateController->out, rotateController->max_out);
  
    return rotateController->out;
  }
  else
  {
    return 0;
  }    
}