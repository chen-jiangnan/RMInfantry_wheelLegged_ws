#include "kalman.h"

void KalmanFilter_Init(KalmanFilterHandle_t* kalmanfliter, uint16_t nx, uint16_t nz, uint16_t nu, float32_t dt)
{
    if(kalmanfliter == NULL){
        return;
    }
    memset(kalmanfliter, 0, sizeof(*kalmanfliter));

    if(nu){
        kalmanfliter->config.ifInput = 1;
        kalmanfliter->Un.nRows = nu;
        kalmanfliter->Un.nColumns = 1;
    }
    kalmanfliter->config.dt = dt;

    kalmanfliter->F.nRows = nx;
    kalmanfliter->F.nColumns = nx;
    kalmanfliter->G.nRows = nx;
    kalmanfliter->G.nColumns = nu;
    kalmanfliter->H.nRows = nz;
    kalmanfliter->H.nColumns = nx;

    kalmanfliter->Xn.nRows = nx;
    kalmanfliter->Xn.nColumns = 1;
    kalmanfliter->Un.nRows = nu;
    kalmanfliter->Un.nColumns = 1;        
    kalmanfliter->Zn.nRows = nz;
    kalmanfliter->Zn.nColumns = 1;   

    kalmanfliter->Q.nRows = nx;
    kalmanfliter->Q.nColumns = nx;
    kalmanfliter->R.nRows = nz;
    kalmanfliter->R.nColumns = nz;        
    kalmanfliter->P.nRows = nx;
    kalmanfliter->P.nColumns = nx;  

    kalmanfliter->Kn.nRows = nx;
    kalmanfliter->Kn.nColumns = nz;
    kalmanfliter->Xn_hat.nRows = nx;
    kalmanfliter->Xn_hat.nColumns = 1; 
    kalmanfliter->Xn_hat_next.nRows = nx;
    kalmanfliter->Xn_hat_next.nColumns = 1; 
}

void KalmanFilter_Update(KalmanFilterHandle_t* kalmanfilter, float32_t* xMatrixData_input, float32_t* uMatrixData_input)
{
    if(kalmanfilter == NULL && xMatrixData_input == NULL){
        return;
    }else{
        arm_mat_init_f32(&kalmanfilter->Xn.matrixHandle, kalmanfilter->Xn.matrixHandle.numRows, kalmanfilter->Xn.matrixHandle.numCols, xMatrixData_input);
    }

    if(kalmanfilter->config.ifInput!=0 &&  uMatrixData_input == NULL){
        return;
    }else if(kalmanfilter->config.ifInput!=0 &&  uMatrixData_input != NULL){
        arm_mat_init_f32(&kalmanfilter->Un.matrixHandle, kalmanfilter->Un.matrixHandle.numRows, kalmanfilter->Un.matrixHandle.numCols, uMatrixData_input);
    }

    arm_matrix_instance_f32 I_matrixHandle;
    arm_matrix_instance_f32 HT_matrixHandle;
    arm_matrix_instance_f32 FT_matrixHandle;
    float32_t  I_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t HT_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t FT_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    arm_mat_init_f32(&I_matrixHandle,  kalmanfilter->Kn.nRows, kalmanfilter->H.nColumns, I_matrixData);
    arm_mat_init_f32(&HT_matrixHandle, kalmanfilter->H.nColumns, kalmanfilter->H.nRows,  HT_matrixData);
    arm_mat_init_f32(&FT_matrixHandle, kalmanfilter->F.nColumns, kalmanfilter->F.nRows,  FT_matrixData);
    for(int i = 0, j = 0; i < I_matrixHandle.numRows; i++, j++){
         I_matrixData[i*I_matrixHandle.numCols + j] = 1;
    }
    arm_mat_trans_f32(&kalmanfilter->H.matrixHandle, &HT_matrixHandle);
    arm_mat_trans_f32(&kalmanfilter->F.matrixHandle, &FT_matrixHandle);

    /*Measurement Update("Correnct")*/
    // 1.Compute the kalman Gain
    // Kn = P(n,n-1)*H^T*(H*P(n,n-1)*H^T + R)^-1
    arm_matrix_instance_f32 out11_matrixHandle;
    arm_matrix_instance_f32 out12_matrixHandle;
    arm_matrix_instance_f32 out13_matrixHandle;
    arm_matrix_instance_f32 out14_matrixHandle;
    arm_matrix_instance_f32 out15_matrixHandle;
    arm_matrix_instance_f32 out16_matrixHandle;
    float32_t out11_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t out12_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t out13_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t out14_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t out15_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t out16_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    arm_mat_init_f32(&out11_matrixHandle,  kalmanfilter->P.matrixHandle.numRows, HT_matrixHandle.numCols, out11_matrixData);
    arm_mat_init_f32(&out12_matrixHandle,  kalmanfilter->H.matrixHandle.numRows, out11_matrixHandle.numCols, out12_matrixData);
    arm_mat_init_f32(&out13_matrixHandle,  kalmanfilter->R.matrixHandle.numRows, kalmanfilter->R.matrixHandle.numCols, out13_matrixData);
    arm_mat_init_f32(&out14_matrixHandle,  out13_matrixHandle.numRows, out13_matrixHandle.numCols, out14_matrixData);
    arm_mat_init_f32(&out15_matrixHandle,  HT_matrixHandle.numRows, out14_matrixHandle.numCols, out15_matrixData);
    arm_mat_init_f32(&out16_matrixHandle,  kalmanfilter->P.matrixHandle.numRows, out15_matrixHandle.numCols, out16_matrixData);

    arm_mat_mult_f32(&kalmanfilter->P.matrixHandle, &HT_matrixHandle, &out11_matrixHandle);
    arm_mat_mult_f32(&kalmanfilter->H.matrixHandle, &out11_matrixHandle, &out12_matrixHandle);
    arm_mat_add_f32(&out12_matrixHandle, &kalmanfilter->R.matrixHandle,  &out13_matrixHandle);
    arm_mat_inverse_f32(&out13_matrixHandle, &out14_matrixHandle);
    arm_mat_mult_f32(&HT_matrixHandle, &out14_matrixHandle, &out15_matrixHandle);
    arm_mat_mult_f32(&kalmanfilter->P.matrixHandle, &out15_matrixHandle, &kalmanfilter->Kn.matrixHandle);
    

    // 2.Update estimate with measurement
    // X^(n,n) = X^(n,n-1) + Kn*(Zn - H*X^(n,n-1))
    arm_matrix_instance_f32 out21_matrixHandle;
    arm_matrix_instance_f32 out22_matrixHandle;
    arm_matrix_instance_f32 out23_matrixHandle;
    float32_t out21_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t out22_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t out23_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    arm_mat_init_f32(&out21_matrixHandle,  kalmanfilter->H.matrixHandle.numRows, kalmanfilter->Xn_hat_next.matrixHandle.numCols, out21_matrixData);
    arm_mat_init_f32(&out22_matrixHandle,  kalmanfilter->Zn.matrixHandle.numRows, kalmanfilter->Zn.matrixHandle.numCols, out22_matrixData);
    arm_mat_init_f32(&out23_matrixHandle,  kalmanfilter->Kn.matrixHandle.numRows, kalmanfilter->Kn.matrixHandle.numCols, out23_matrixData);

    arm_mat_mult_f32(&kalmanfilter->H.matrixHandle, &kalmanfilter->Xn_hat_next.matrixHandle, &out21_matrixHandle);
    arm_mat_sub_f32(&kalmanfilter->Zn.matrixHandle, &out21_matrixHandle, &out22_matrixHandle);
    arm_mat_mult_f32(&kalmanfilter->Kn.matrixHandle, &out22_matrixHandle, &out23_matrixHandle);
    arm_mat_add_f32(&kalmanfilter->Xn_hat_next.matrixHandle, &out23_matrixHandle, &kalmanfilter->Xn_hat.matrixHandle);


    // 3.Update the estimate uncertainty
    // P(n,n) = (I - Kn*H)*P(n,n-1)*(I - Kn*H)^T + Kn*R*Kn^T
    arm_matrix_instance_f32 out31_matrixHandle;
    arm_matrix_instance_f32 out32_matrixHandle;
    arm_matrix_instance_f32 out33_matrixHandle;
    arm_matrix_instance_f32 out34_matrixHandle;
    arm_matrix_instance_f32 out35_matrixHandle;
    arm_matrix_instance_f32 out36_matrixHandle;
    arm_matrix_instance_f32 out37_matrixHandle;
    arm_matrix_instance_f32 out38_matrixHandle;
    float32_t out31_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t out32_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t out33_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t out34_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t out35_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t out36_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t out37_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t out38_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    arm_mat_init_f32(&out31_matrixHandle,  kalmanfilter->Kn.matrixHandle.numRows, kalmanfilter->H.matrixHandle.numCols, out31_matrixData);
    arm_mat_init_f32(&out32_matrixHandle,  I_matrixHandle.numRows, out31_matrixHandle.numCols, out32_matrixData);
    arm_mat_init_f32(&out33_matrixHandle,  out32_matrixHandle.numRows, out32_matrixHandle.numCols, out33_matrixData);
    arm_mat_init_f32(&out34_matrixHandle,  kalmanfilter->P.matrixHandle.numRows, out33_matrixHandle.numCols, out34_matrixData);     
    arm_mat_init_f32(&out35_matrixHandle,  out32_matrixHandle.numRows, out34_matrixHandle.numCols, out35_matrixData);
    arm_mat_init_f32(&out36_matrixHandle,  kalmanfilter->Kn.matrixHandle.numRows, kalmanfilter->Kn.matrixHandle.numCols, out36_matrixData);
    arm_mat_init_f32(&out37_matrixHandle,  kalmanfilter->R.matrixHandle.numRows, out36_matrixHandle.numCols, out37_matrixData);
    arm_mat_init_f32(&out38_matrixHandle,  kalmanfilter->Kn.matrixHandle.numRows, out37_matrixHandle.numCols, out38_matrixData);   

    arm_mat_mult_f32(&kalmanfilter->Kn.matrixHandle, &kalmanfilter->H.matrixHandle, &out31_matrixHandle);
    arm_mat_sub_f32(&I_matrixHandle, &out31_matrixHandle, &out32_matrixHandle);
    arm_mat_trans_f32(&out32_matrixHandle, &out33_matrixHandle);
    arm_mat_mult_f32(&kalmanfilter->P.matrixHandle, &out33_matrixHandle, &out34_matrixHandle);
    arm_mat_mult_f32(&out32_matrixHandle, &out34_matrixHandle, &out35_matrixHandle);

    arm_mat_trans_f32(&kalmanfilter->Kn.matrixHandle, &out36_matrixHandle);
    arm_mat_mult_f32(&kalmanfilter->R.matrixHandle, &out36_matrixHandle, &out37_matrixHandle);
    arm_mat_mult_f32(&kalmanfilter->Kn.matrixHandle, &out37_matrixHandle, &out38_matrixHandle);

    arm_mat_add_f32(&out35_matrixHandle, &out38_matrixHandle, &kalmanfilter->P.matrixHandle);


    /*Time Update("Predict")*/
    // 4.Extrapolate the state
    // X^(n+1, n) = F*X^(n,n) + G*Un
    if(kalmanfilter->config.ifInput != 0){
        arm_matrix_instance_f32 out41_matrixHandle;
        arm_matrix_instance_f32 out42_matrixHandle;
        float32_t out41_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
        float32_t out42_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
        arm_mat_init_f32(&out41_matrixHandle,  kalmanfilter->F.matrixHandle.numRows, kalmanfilter->Xn_hat.nColumns, out41_matrixData);
        arm_mat_init_f32(&out42_matrixHandle,  kalmanfilter->G.matrixHandle.numRows, kalmanfilter->Un.matrixHandle.numCols, out42_matrixData);

        arm_mat_mult_f32(&kalmanfilter->F.matrixHandle, &kalmanfilter->Xn_hat.matrixHandle, &out41_matrixHandle);
        arm_mat_mult_f32(&kalmanfilter->G.matrixHandle, &kalmanfilter->Un.matrixHandle, &out42_matrixHandle);
        arm_mat_add_f32(&out41_matrixHandle, &out42_matrixHandle, &kalmanfilter->Xn_hat_next.matrixHandle);
    }else{
        arm_mat_mult_f32(&kalmanfilter->F.matrixHandle, &kalmanfilter->Xn_hat.matrixHandle, &kalmanfilter->Xn_hat_next.matrixHandle);  
    }



    // 5.Extrapolate uncertainty
    // P(n+1, n) = F*P(n,n)*F^T + Q
    arm_matrix_instance_f32 out51_matrixHandle;
    arm_matrix_instance_f32 out52_matrixHandle;
    float32_t out51_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    float32_t out52_matrixData[KALMAN_MATRIX_SIZE*KALMAN_MATRIX_SIZE] = {0};
    arm_mat_init_f32(&out51_matrixHandle,  kalmanfilter->P.matrixHandle.numRows, FT_matrixHandle.numCols, out51_matrixData);
    arm_mat_init_f32(&out52_matrixHandle,  kalmanfilter->F.matrixHandle.numRows, out51_matrixHandle.numCols, out52_matrixData);

    arm_mat_mult_f32(&kalmanfilter->P.matrixHandle, &FT_matrixHandle, &out51_matrixHandle);
    arm_mat_mult_f32(&kalmanfilter->F.matrixHandle, &out51_matrixHandle, &out52_matrixHandle);
    arm_mat_add_f32(&out52_matrixHandle, &kalmanfilter->Q.matrixHandle, &kalmanfilter->P.matrixHandle);
}