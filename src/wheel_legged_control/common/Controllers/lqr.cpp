#include "lqr.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <ostream>

float LQRKMatrixData[12] = {
//  -45.3562/4,   -7.5698,  -21.9805*0,  18.6475/10,   16.7936/4,    1.4572, //T
//   14.8143,    2.4898,    8.2117,    6.6742,  139.8843/8,    3.6386, //Tp
//    0,  0,  0,  -1,   18,   2.5, //T
//   14.8143,    2.4898,    8.2117,    6.6742,  139.8843/8,    3.6386, //Tp
//     -5,  -1,  0,  -2,   18,   2.5, //T
//    14.8143,    2.4898,    8.2117,    9,  100.0f,    3.6386, //Tp
// -23.0079,   -2.8830,   -0.2936 ,  -6.6799   ,10.9383 ,   1.0311,
// 14.2645,    1.9935,    0.2348  ,  5.3336  , 42.7219   , 1.8517,
-26.5178,   -3.5127,   -0.2898,   -8.3048,   11.9932,    1.1634,
18.0939,    2.6677,    0.2533,    7.2507,   41.7376,    1.7153,
};//  theta  theta_dot    x       x_dot     phi     phi_dot

/**
 * @brief       LQR controller init
 * @param[out]  lqr_controller:lqr controller handle
 * @return      None   
 * 
*/
void LQRControllerInit(LQRControllerHandle_t* lqr_controller)
{   /*lqr gain*/
    lqr_controller->Gain.matrixHandle.setZero();
    lqr_controller->Gain.matrixHandle = Eigen::Map<Eigen::Matrix<float, 2, 6, Eigen::RowMajor>>(LQRKMatrixData);
    /*lqr x*/
    lqr_controller->X.matrixHandle.setZero();
    /*lqr xd*/
    lqr_controller->Xd.matrixHandle.setZero();
    /*lqr err*/
    lqr_controller->Err.matrixHandle.setZero();
    /*lqr out*/
    lqr_controller->Out.matrixHandle.setZero();    
}


/**
 * @brief       LQR controller calculate
 * @note        out = lqr_gain*(xd - x)
 * @param[out]   lqr_controller:lqr controller handle
 * @param[in]    xd:Xd Matrix(the target system state)
 * @param[in]    x:X Matrix(the real-time system state)
 * @param[out]   out:Out Matrix(the lqr controller output value)
 * @return      None   
 * 
*/
void LQRController_Calc(LQRControllerHandle_t* lqr_controller, float* xd, float* x, float* out)
{
    /*input*/
    // memcpy(lqr_controller->Xd.matrixData, xd, 6*sizeof(float));
    // memcpy(lqr_controller->X.matrixData, x, 6*sizeof(float));
    lqr_controller->Xd.matrixHandle = Eigen::Map<Eigen::VectorXf>(xd, 6);
    lqr_controller->X.matrixHandle = Eigen::Map<Eigen::VectorXf>(x, 6);

    /*xd-x*/
    lqr_controller->Err.matrixHandle = lqr_controller->Xd.matrixHandle - lqr_controller->X.matrixHandle;

    /*out = LQR_K*(xd-x)*/
    lqr_controller->Out.matrixHandle = lqr_controller->Gain.matrixHandle * lqr_controller->Err.matrixHandle;

    /*output*/
    memcpy(out, lqr_controller->Out.matrixHandle.data(), 6*sizeof(float));
}
