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
        float theta[3];
        float L0[3];
        float ddot_zb;

        float ddot_L0;
        float dot_L0;
        float ddot_theta;
        float dot_theta;
        
        float ddot_zw;
        float Fn;
    }FnEst[2];
}StateEstimatorHandle_t;


void stateEstimatorInit(StateEstimatorHandle_t* stateEstimator);
void FnEstimateUpdate(StateEstimatorHandle_t* stateEstimator, 
    const float* theta, 
    const float* L0,  
    const float* F, 
    const float* Tp,
    float ddot_zb,
    float mw, 
    float dt
);
// bool checkContact(StateEstimatorHandle_t* stateEstimator);

#endif
  