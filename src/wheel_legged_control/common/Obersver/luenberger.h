#ifndef __LUENBERGER_H
#define __LUENBERGER_H

#include "arm_math.h"


typedef struct {
    /*define state-space*/
    struct{
        arm_matrix_instance_f32 A_matrixHandle;  //system matrix
        arm_matrix_instance_f32 B_matrixHandle;  //input matrix
        arm_matrix_instance_f32 C_matrixHandle;  //output  matrix
        arm_matrix_instance_f32 D_matrixHandle;  //direct transmission matrix
    }sysSpace;

    /*define observer system state*/
    struct{
        arm_matrix_instance_f32 x_matrixHandle;  //system state
        arm_matrix_instance_f32 y_matrixHandle;  //system output vector
        arm_matrix_instance_f32 u_matrixHandle;  //system input vector 
    }sysState;   

    /*define observer system state*/
    struct{
        arm_matrix_instance_f32 xHat_matrixHandle;
        arm_matrix_instance_f32 xDotHat_matrixHandle;
        arm_matrix_instance_f32 yHat_matrixHandle;  //output vector
    }observerState;

    /*define luenberger gain matrix*/
    arm_matrix_instance_f32 L_matrixHandle;

}LuenbergerObserverInterface_t;


#endif
