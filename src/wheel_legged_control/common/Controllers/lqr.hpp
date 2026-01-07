#ifndef __LQR_H
#define __LQR_H

#include <Eigen/Dense>
#include <cstdint>
#include <Eigen/src/Core/Matrix.h>
extern float LQRKMatrixData[12];
 typedef struct{
    struct{
        Eigen::MatrixXf matrixHandle{2,6};
        // float matrixData[2*6];
    }Gain;

    struct{
        // float matrixData[6];//[theta;theta_dot;x;x_dot;phi;phi_dot] 
        Eigen::VectorXf matrixHandle{6};
    }X;

    struct {
        // float matrixData[6];//[theta;theta_dot;x;x_dot;phi;phi_dot] 
        Eigen::VectorXf matrixHandle{6};
    }Xd;

    struct {
        Eigen::VectorXf matrixHandle{6};
        // float matrixData[6];// xd - x
    }Err;  

    struct{
        Eigen::VectorXf matrixHandle{2};
        // float matrixData[2*1];//[Tp;T]
    }Out;

    // struct{
    //     Eigen::MatrixXf matrixHandle;
    //     float matrixData[6*6];
    // }Q;

    // struct{
    //     Eigen::MatrixXf matrixHandle;
    //     float matrixData[2*2];        
    // }R;
    
 }LQRControllerHandle_t;

/**
 * @brief       LQR controller init
 * @param[in]   lqr_controller:lqr controller handle
 * @return      None   
 * 
*/
void LQRControllerInit(LQRControllerHandle_t* lqr_controller);


/**
 * @brief       LQR controller calculate
 * @note        out = lqr_gain*(xd - x)
 * @param[in]   lqr_controller:lqr controller handle
 * @param[in]   xd:Xd Matrix(the target system state)
 * @param[in]   x:X Matrix(the real-time system state)
 * @param[in]   out:Out Matrix(the lqr controller output value)
 * @return      None   
 * 
*/
void LQRController_Calc(LQRControllerHandle_t* lqr_controller, float* xd, float* x, float* out);

#endif
