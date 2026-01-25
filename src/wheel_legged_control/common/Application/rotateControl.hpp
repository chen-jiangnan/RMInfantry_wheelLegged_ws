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
#ifndef __ROTATECONTROL_H
#define __ROTATECONTROL_H

#include "Controllers/pid.hpp"
#define ROTATECONTROL_SPEEDPID_POSITION_KP 3.0f
#define ROTATECONTROL_SPEEDPID_POSITION_KI 0.0f
#define ROTATECONTROL_SPEEDPID_POSITION_KD 0.5



/** 
  * @brief          init RotateController handle
  * @param[in]      rotateController: handle address
  * @retval         none
  */
void rotateControllerInit(pid_type_def* rotateController);


/** 
  * @brief          iotateController calculate
  * @param[in]      rotateController:rotateControoler handle
  * @param[in]      target_wYaw:目标角速度
  * @param[in]      rev_wYaw:机体角速度反馈
  * @retval         none
  */
float rotateControl(pid_type_def* rotateController, float target_wYaw, float rev_wYaw);
#endif
  