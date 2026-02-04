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
// -26.5178,   -3.5127,   -0.2898,   -8.3048/5,   11.9932,    1.1634,
// 18.0939,    2.6677,    0.2533,    7.2507/3,   41.7376,    1.7153,
// -12.6792,   -0.9357,   -1.2094,   -1.6544*0,    3.9824,    0.4786,
// 14.2672,    1.2053,    2.4408,    3.1888*0,   13.1909,    0.8906,

// -19.7031,   -1.3284,   -1.1472,   -1.9152,    5.1593,    0.6739,
// 25.0396,    1.9053,    2.7140,    4.3485,   13.5465,    1.1088
-45.0133,  -6.0558,  -21.2825,  -17.7892,   26.8377,    2.1362,
23.7963,    3.4670,   13.7196 ,   11.0698,  136.5437,    4.6680,
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
