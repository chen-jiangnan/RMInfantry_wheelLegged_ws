#include "luenberger.h"

void LuenbergerObserver_Init(LuenbergerObserverInterface_t* lg_observer, uint16_t num_x, uint16_t num_u)
{
    lg_observer->sysSpace.A_matrixHandle.numRows = num_x;
    lg_observer->sysSpace.A_matrixHandle.numCols = num_x;
}

void LuenbergerObserver_sysSpaceInit(
    LuenbergerObserverInterface_t* lg_observer,
    float32_t* A_matrixData,
    float32_t* B_matrixData,
    float32_t* C_matrixData,
    float32_t* D_matrixData
){
    if(lg_observer == NULL || A_matrixData == NULL || C_matrixData == NULL){
        return;
    }
    arm_mat_init_f32(&lg_observer->sysSpace.A_matrixHandle, lg_observer->sysSpace.A_matrixHandle.numRows, lg_observer->sysSpace.A_matrixHandle.numCols, A_matrixData);
    arm_mat_init_f32(&lg_observer->sysSpace.C_matrixHandle, lg_observer->sysSpace.C_matrixHandle.numRows, lg_observer->sysSpace.C_matrixHandle.numCols, C_matrixData);
    arm_mat_init_f32(&lg_observer->sysSpace.B_matrixHandle, lg_observer->sysSpace.B_matrixHandle.numRows, lg_observer->sysSpace.B_matrixHandle.numCols, B_matrixData);
    arm_mat_init_f32(&lg_observer->sysSpace.D_matrixHandle, lg_observer->sysSpace.D_matrixHandle.numRows, lg_observer->sysSpace.D_matrixHandle.numCols, D_matrixData);
}

void LuenbergerObserver_sysStateInit(
    LuenbergerObserverInterface_t* lg_observer,
    float32_t* x_matrixData,
    float32_t* u_matrixData,
    float32_t* y_matrixData
){
    if(lg_observer == NULL || x_matrixData == NULL || y_matrixData == NULL || u_matrixData == NULL){
        return;
    }
    arm_mat_init_f32(&lg_observer->sysState.x_matrixHandle, lg_observer->sysState.x_matrixHandle.numRows, lg_observer->sysState.x_matrixHandle.numCols, x_matrixData);
    arm_mat_init_f32(&lg_observer->sysState.y_matrixHandle, lg_observer->sysState.y_matrixHandle.numRows, lg_observer->sysState.y_matrixHandle.numCols, y_matrixData);
    arm_mat_init_f32(&lg_observer->sysState.u_matrixHandle, lg_observer->sysState.u_matrixHandle.numRows, lg_observer->sysState.u_matrixHandle.numCols, u_matrixData);
}

void LuenbergerObserver_observerStateInit(
    LuenbergerObserverInterface_t* lg_observer,
    float32_t* xhat_matrixData,
    float32_t* xdothat_matrixData,
    float32_t* yhat_matrixData
){
    if(lg_observer == NULL || xhat_matrixData == NULL || xdothat_matrixData == NULL || yhat_matrixData == NULL){
        return;
    }
    arm_mat_init_f32(&lg_observer->observerState.xHat_matrixHandle, lg_observer->observerState.xHat_matrixHandle.numRows, lg_observer->observerState.xHat_matrixHandle.numCols, xhat_matrixData);
    arm_mat_init_f32(&lg_observer->observerState.yHat_matrixHandle, lg_observer->observerState.yHat_matrixHandle.numRows, lg_observer->observerState.yHat_matrixHandle.numCols, yhat_matrixData);
    arm_mat_init_f32(&lg_observer->observerState.xDotHat_matrixHandle, lg_observer->observerState.xDotHat_matrixHandle.numRows, lg_observer->observerState.xDotHat_matrixHandle.numCols, xdothat_matrixData);
}

void LuenbergerObserver_gainConfigInit(    
    LuenbergerObserverInterface_t* lg_observer,
    float32_t* L_matrixData
){
    if(lg_observer == NULL || L_matrixData == NULL ){
        return;
    }
    arm_mat_init_f32(&lg_observer->L_matrixHandle, lg_observer->L_matrixHandle.numRows, lg_observer->L_matrixHandle.numCols, L_matrixData);   
}

void LuenbergerObserver_Calculate(LuenbergerObserverInterface_t* lg_observer)
{
    // y^n-1 = C*x^n-1
    arm_mat_mult_f32(&lg_observer->sysSpace.C_matrixHandle, &lg_observer->observerState.xHat_matrixHandle, &lg_observer->observerState.yHat_matrixHandle);

    /*compulate system output*/
    // yn = C*xn
    arm_mat_mult_f32(&lg_observer->sysSpace.C_matrixHandle, &lg_observer->sysState.x_matrixHandle, &lg_observer->sysState.y_matrixHandle);

    /*luenberger observer feedback loop*/
    //L(yn-1-y^n-1)
    arm_matrix_instance_f32 error_matrixHandle;
    const uint16_t e_numRows = lg_observer->sysState.y_matrixHandle.numRows;
    const uint16_t e_numCols = lg_observer->sysState.y_matrixHandle.numCols;
    float32_t error_matrixDate[e_numRows*e_numCols];
    arm_mat_init_f32(&error_matrixHandle, e_numRows, e_numCols, error_matrixDate);
    arm_mat_sub_f32(&lg_observer->sysState.y_matrixHandle, &lg_observer->observerState.yHat_matrixHandle, &error_matrixHandle);

    arm_matrix_instance_f32 temp_matrixHandle;
    const uint16_t t_numRows = lg_observer->L_matrixHandle.numRows;
    const uint16_t t_numCols = lg_observer->sysState.x_matrixHandle.numCols;
    float32_t temp_matrixData[t_numRows*t_numCols];
    arm_mat_init_f32(&temp_matrixHandle, t_numRows, t_numCols, temp_matrixData);
    arm_mat_mult_f32(&lg_observer->L_matrixHandle, &error_matrixHandle, &temp_matrixHandle);

    /*compulate observer state*/
    // X^n = A*X^n-1 + B*Ux-1 + L(yn-1 - y^n-1)
    if(lg_observer->sysSpace.B_matrixHandle.pData != NULL){
        arm_matrix_instance_f32 temp1_matrixHandle;
        const uint16_t t1_numRows = lg_observer->sysSpace.A_matrixHandle.numRows;
        const uint16_t t1_numCols = lg_observer->observerState.xHat_matrixHandle.numCols;
        float32_t temp1_matrixData[t1_numRows*t1_numCols];
        arm_mat_init_f32(&temp1_matrixHandle, t1_numRows, t1_numCols, temp1_matrixData);
        arm_mat_mult_f32(&lg_observer->sysSpace.A_matrixHandle, &lg_observer->observerState.xHat_matrixHandle, &temp1_matrixHandle);

        arm_matrix_instance_f32 temp2_matrixHandle;
        const uint16_t t2_numRows = lg_observer->sysSpace.B_matrixHandle.numRows;
        const uint16_t t2_numCols = lg_observer->sysState.u_matrixHandle.numCols;
        float32_t temp2_matrixData[t2_numRows*t2_numCols];
        arm_mat_init_f32(&temp2_matrixHandle, t2_numRows, t2_numCols, temp2_matrixData);
        arm_mat_mult_f32(&lg_observer->sysSpace.B_matrixHandle, &lg_observer->sysState.u_matrixHandle, &temp2_matrixHandle);

        arm_matrix_instance_f32 temp3_matrixHandle;
        const uint16_t t3_numRows = t2_numRows;
        const uint16_t t3_numCols = t2_numCols;
        float32_t temp3_matrixData[t3_numRows*t3_numCols];
        arm_mat_add_f32(&temp1_matrixHandle, &temp2_matrixHandle, &temp3_matrixHandle);
        arm_mat_add_f32(&temp3_matrixHandle, &temp_matrixHandle, &lg_observer->observerState.xDotHat_matrixHandle);

    }else{
        arm_matrix_instance_f32 temp1_matrixHandle;
        const uint16_t t1_numRows = lg_observer->sysSpace.A_matrixHandle.numRows;
        const uint16_t t1_numCols = lg_observer->observerState.xHat_matrixHandle.numCols;
        float32_t temp1_matrixData[t1_numRows*t1_numCols];
        arm_mat_init_f32(&temp1_matrixHandle, t1_numRows, t1_numCols, temp1_matrixData);
        arm_mat_mult_f32(&lg_observer->sysSpace.A_matrixHandle, &lg_observer->observerState.xHat_matrixHandle, &temp1_matrixHandle);

        arm_matrix_instance_f32 temp2_matrixHandle;
        const uint16_t t2_numRows = lg_observer->L_matrixHandle.numRows;
        const uint16_t t2_numCols = lg_observer->observerState.yHat_matrixHandle.numCols;
        float32_t temp2_matrixData[t2_numRows*t2_numCols];
        arm_mat_init_f32(&temp2_matrixHandle, t2_numRows, t2_numCols, temp2_matrixData);

        arm_mat_add_f32(&temp1_matrixHandle, &temp_matrixHandle, &lg_observer->observerState.xDotHat_matrixHandle);
    }
}