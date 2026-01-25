#ifndef __KALMAN_H
#define __KALMAN_H

#include "arm_math.h"

#define KALMAN_MATRIX_SIZE 4
typedef struct{
    /*系统模型*/
    struct{
        arm_matrix_instance_f32 matrixHandle;
        uint16_t nRows; 
        uint16_t nColumns;
    }F;// 状态转移矩阵
    struct{
        arm_matrix_instance_f32 matrixHandle;
        uint16_t nRows; 
        uint16_t nColumns;
    }G;// 控制矩阵

    /*观测模型*/
    struct{
        arm_matrix_instance_f32 matrixHandle;
        uint16_t nRows; 
        uint16_t nColumns;        
    }H;// 观测矩阵

    /*系统状态与系统输入*/
    struct{
        arm_matrix_instance_f32 matrixHandle;
        uint16_t nRows; 
        uint16_t nColumns;        
    }Xn;// 系统n时刻状态矩阵
    struct{
        arm_matrix_instance_f32 matrixHandle;
        uint16_t nRows; 
        uint16_t nColumns;             
    }Un;// 系统n时刻输入矩阵
    struct{
        arm_matrix_instance_f32 matrixHandle;
        uint16_t nRows; 
        uint16_t nColumns;             
    }Zn;// 系统n时刻观测值

    /*噪声*/
    struct{
        arm_matrix_instance_f32 matrixHandle;
        uint16_t nRows; 
        uint16_t nColumns;        
    }Q;// 过程噪声协方差矩阵
    struct{
        arm_matrix_instance_f32 matrixHandle;
        uint16_t nRows; 
        uint16_t nColumns;             
    }R;// 测量噪声协方差矩阵
    struct{
        arm_matrix_instance_f32 matrixHandle;
        uint16_t nRows; 
        uint16_t nColumns;             
    }P;// 估计协方差矩阵

    /*kalman 相关参数*/
    struct{
        arm_matrix_instance_f32 matrixHandle;
        uint16_t nRows; 
        uint16_t nColumns;           
    }Kn;// kalman n时刻增益
    struct{
        arm_matrix_instance_f32 matrixHandle;
        uint16_t nRows; 
        uint16_t nColumns;           
    }Xn_hat;// 对n+1时刻的预测值 
    struct{
        arm_matrix_instance_f32 matrixHandle;
        uint16_t nRows; 
        uint16_t nColumns;           
    }Xn_hat_next;// 对n+1时刻的预测值 


    struct{
        float32_t dt;    //步长
        uint8_t ifInput; //bool值 0为无输入kalman, 非零为有输入kalman
    }config;    
}KalmanFilterHandle_t;


#endif