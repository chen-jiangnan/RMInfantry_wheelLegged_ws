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
#ifndef __STATEESTIMATE_H
#define __STATEESTIMATE_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

typedef struct{
    struct{
        float alpha[3];        
        float theta[3];
        float L0[3];

        float ddot_L0;
        float dot_L0;
        float ddot_theta;
        float dot_theta;
        float ddot_alpha;
        float dot_alpha;
    }base[2];

    struct{
        float ddot_zb;
        float ddot_zw[2];
        float Fn[2];
    }FnEst;

    struct{
        float w_ecd[2];
        float b_acc;

        float dot_xw[2];//驱动轮相对于大地的线速度
        float aver_vel;

        float x_filter;
        float v_filter;

    }xvEst;
    
    float dt;
}StateEstimatorHandle_t;


void stateEstimatorInit(StateEstimatorHandle_t* stateEstimator, float dt);
void stateEstimatorUpdate(StateEstimatorHandle_t* stateEstimator,
    const float* alpha, 
    const float* theta, 
    const float* L0
);

void FnEstimateCalc(StateEstimatorHandle_t* stateEstimator, 
    const float* F, 
    const float* Tp,
    float ddot_zb,
    float mw
);
bool groundDetection(StateEstimatorHandle_t* stateEstimator);

void xvEstimateCalc(StateEstimatorHandle_t* stateEstimator,
    const float* w_ecd, 
    float b_acc,
    float pitch,      
    float Rw 
);
// bool checkContact(StateEstimatorHandle_t* stateEstimator);

#endif
  