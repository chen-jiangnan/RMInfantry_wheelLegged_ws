#ifndef __LEGCONTROL_H
#define __LEGCONTROL_H

#include "Controllers/pid.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

#define LEGCONTROL_SPRINGDAMPING_PID_POSITION_KP 800.0f
#define LEGCONTROL_SPRINGDAMPING_PID_POSITION_KI   0.0f
#define LEGCONTROL_SPRINGDAMPING_PID_POSITION_KD  80.0f

#define LEGCONTROL_COMPENSATIONROLL_PID_POSITION_KP    -0.0f
#define LEGCONTROL_COMPENSATIONROLL_PID_POSITION_KI    0.0f
#define LEGCONTROL_COMPENSATIONROLL_PID_POSITION_KD    0.0f

#define LEGCONTROL_COMPENSATIONPHI0_PID_POSITION_KP    2.0f
#define LEGCONTROL_COMPENSATIONPHI0_PID_POSITION_KI    0.1f
#define LEGCONTROL_COMPENSATIONPHI0_PID_POSITION_KD    0.0f

#define LEGCONTROL_LEG_LENGTH_MAX 0.32 // 单位：m
#define LEGCONTROL_LEG_LENGTH_MIN 0.15 // 单位: m

typedef struct{

  struct{
    pid_type_def springDamping;     //vmc弹簧阻尼控制器
    float feedforward_gravity;      //抵消机体自重的力源
    float feedforward_fs;           //机体跳跃的额外力源
    float F;                        //总输出 F = springDamping.out + compensationRoll.out + feedforward_gravity + feedforward_fs
	  float compensation_tp;
  }base[2];
  pid_type_def compensationRoll;    //机体roll角补偿控制器
  pid_type_def compensationPhi0;    //theta补偿控制器  
}LegControllerHandle_t;


/** 
  * @brief          init legController handle
  * @param[in]      legcontroller: handle address
  * @retval         none
  */
void legControllerInit(LegControllerHandle_t* legcontroller);


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
);
#endif
