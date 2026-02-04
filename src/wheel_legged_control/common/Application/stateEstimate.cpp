#include "stateEstimate.hpp"
#include "Filter/kalman_filter.hpp"
#include <cmath>
#include <iostream>


KalmanFilter vaEstimateKF(2, 0, 2); // 卡尔曼滤波器结构体

void stateEstimatorInit(StateEstimatorHandle_t* stateEstimator, float dt){
    memset(stateEstimator, 0, sizeof(StateEstimatorHandle_t));
    stateEstimator->dt = dt;
    vaEstimateKF.F << 1.0f, dt,
                      0.0f, 1.0f;
    vaEstimateKF.P << 1.0f, 0.0f,
                      0.0f, 1.0f;
    vaEstimateKF.Q << 0.5f, 0.0f,
                      0.0f, 0.5f;
    vaEstimateKF.R_diag = {100.0f, 100.0f};
    vaEstimateKF.measurement_map = {1, 2};
    vaEstimateKF.measurement_degree = {1, 1};

}

void stateEstimatorUpdate(StateEstimatorHandle_t* stateEstimator,
    const float* alpha, 
    const float* theta, 
    const float* L0
){
    float dt = stateEstimator->dt;
    for(int i = 0; i < 2; i++){
        // update input value
        stateEstimator->base[i].alpha[2] = stateEstimator->base[i].alpha[1];
        stateEstimator->base[i].alpha[1] = stateEstimator->base[i].alpha[0];
        stateEstimator->base[i].alpha[0] = alpha[i];
        stateEstimator->base[i].theta[2] = stateEstimator->base[i].theta[1];
        stateEstimator->base[i].theta[1] = stateEstimator->base[i].theta[0];
        stateEstimator->base[i].theta[0] = theta[i];
        stateEstimator->base[i].L0[2] = stateEstimator->base[i].L0[1];
        stateEstimator->base[i].L0[1] = stateEstimator->base[i].L0[0];
        stateEstimator->base[i].L0[0] = L0[i];

        stateEstimator->base[i].dot_alpha = (stateEstimator->base[i].alpha[0] - stateEstimator->base[i].alpha[1])/dt;
        stateEstimator->base[i].ddot_alpha = (stateEstimator->base[i].alpha[0] - 2*stateEstimator->base[i].alpha[1] + stateEstimator->base[i].alpha[2])/(dt*dt);
        stateEstimator->base[i].dot_theta = (stateEstimator->base[i].theta[0] - stateEstimator->base[i].theta[1])/dt;
        stateEstimator->base[i].ddot_theta = (stateEstimator->base[i].theta[0] - 2*stateEstimator->base[i].theta[1] + stateEstimator->base[i].theta[2])/(dt*dt);
        stateEstimator->base[i].dot_L0 = (stateEstimator->base[i].L0[0] - stateEstimator->base[i].L0[1])/dt;
        stateEstimator->base[i].ddot_L0 = (stateEstimator->base[i].L0[0] - 2*stateEstimator->base[i].L0[1] + stateEstimator->base[i].L0[2])/(dt*dt); 
    }    
}

void FnEstimateCalc(StateEstimatorHandle_t* stateEstimator, 
    const float* F, 
    const float* Tp,
    float ddot_zb,
    float mw
){
    static float Fn_buffer[2][10] = {{0}};
    static int data_count[2] = {0};
    for(int i = 0; i < 2; i++){
        // update input value
        float theta = stateEstimator->base[i].theta[0];
        float L0 = stateEstimator->base[i].L0[0];
        float dot_theta = stateEstimator->base[i].dot_theta;
        float ddot_theta = stateEstimator->base[i].ddot_theta;
        float dot_L0 = stateEstimator->base[i].dot_L0;
        float ddot_L0 = stateEstimator->base[i].ddot_L0;

        Fn_buffer[i][9] = Fn_buffer[i][8];
        Fn_buffer[i][8] = Fn_buffer[i][7];
        Fn_buffer[i][7] = Fn_buffer[i][6];
        Fn_buffer[i][6] = Fn_buffer[i][5];
        Fn_buffer[i][5] = Fn_buffer[i][4];
        Fn_buffer[i][4] = Fn_buffer[i][3];
        Fn_buffer[i][3] = Fn_buffer[i][2];
        Fn_buffer[i][2] = Fn_buffer[i][1];
        Fn_buffer[i][1] = Fn_buffer[i][0];

        stateEstimator->FnEst.ddot_zb = ddot_zb;
        // P = Fcos(theta) + Tpsin(theta)/L0
        float P = F[i]*std::cos(theta) + Tp[i]*std::sin(theta)/L0;
        // ddot_zw = ddot_zb - ddot_L0*cos(theta) + 2*dot_L0*dot_theta*sin(theta) + L0*ddot_theta*sin(theta) + L0*(d_theta)^2*cos(theta)
        float ddot_zw = ddot_zb - ddot_L0*std::cos(theta) 
                        + 2*dot_L0*dot_theta*std::sin(theta) 
                        + L0*ddot_theta*std::sin(theta)
                        + L0*(dot_theta*dot_theta)*std::cos(theta);  
        float Fn = mw*ddot_zw*0 + mw*9.8 + P;
        // update output value
        Fn_buffer[i][0] = Fn;
        
        if(data_count[i] < 10){
            data_count[i]++;
        }
        
        float Fn_avg = 0;
        for(int j = 0; j < data_count[i]; j++){
            Fn_avg += Fn_buffer[i][j];
        }
        Fn_avg /= data_count[i];
        stateEstimator->FnEst.ddot_zw[i] = ddot_zw;
        stateEstimator->FnEst.Fn[i] = Fn_avg; 

        // std::cout<< "F: "<< F[i] << std::endl;
        // std::cout<< "Tp: "<< Tp[i] << std::endl;
        // std::cout<< "P: "<< P << std::endl;
        // std::cout<< "dot_theta: "<< dot_theta << std::endl;
        // std::cout<< "ddot_theta: "<< ddot_theta << std::endl;
        // std::cout<< "dot_L0: "<< dot_L0 << std::endl;
        // std::cout<< "ddot_L0: "<< ddot_L0 << std::endl;
        // std::cout<< "Fn_avg: "<< Fn_avg << std::endl;
        // std::cout<<"===================================="<<std::endl;
    }    
}

bool groundDetection(StateEstimatorHandle_t* stateEstimator){
    float left_Fn = stateEstimator->FnEst.Fn[0];
    float right_Fn = stateEstimator->FnEst.Fn[1];
    static int ground_time = 0;
    static bool ground_signal = false;
    // 正常支撑力工作区间
    if(( left_Fn > 100.0f && right_Fn > 100.0f) && (left_Fn < 150.0f && right_Fn < 150.0f)){
        if(ground_signal){
            ground_time++;
            if(ground_time > 10){
                ground_signal = false;
            }
        }
    }
    // 离地支撑力工作区间
    else if(left_Fn < 80.0f && right_Fn < 80.0f){
        ground_signal = true;
    }
    // 非正常支撑力工作区间
    else if(left_Fn > 200.0f && right_Fn > 200.0f){
        ground_signal = true;
    }

    return ground_signal;
}

void xvEstimateCalc(StateEstimatorHandle_t* stateEstimator,
    const float* w_ecd, 
    float b_acc,
    float pitch,      
    float Rw 
){
    // update input value
    stateEstimator->xvEst.w_ecd[0] = w_ecd[0];
    stateEstimator->xvEst.w_ecd[1] = w_ecd[1];
    stateEstimator->xvEst.b_acc = b_acc;

    float dt = stateEstimator->dt;

    for(int i = 0; i < 2; i++){
        float L0 = stateEstimator->base[i].L0[0]; 
        float theta = stateEstimator->base[i].theta[0];

        float dot_L0 = stateEstimator->base[i].dot_L0;
        float dot_theta = stateEstimator->base[i].dot_theta;
        float dot_alpha = stateEstimator->base[i].dot_alpha;

        float w = stateEstimator->xvEst.w_ecd[i] + pitch +  dot_alpha;
        stateEstimator->xvEst.dot_xw[i] = w*Rw  ;//+ L0*dot_theta*std::cos(theta) + dot_L0*std::sin(theta)
    }
    stateEstimator->xvEst.aver_vel = (stateEstimator->xvEst.dot_xw[0] + stateEstimator->xvEst.dot_xw[1])/2.0f;

    //卡尔曼滤波器测量值更新
    vaEstimateKF.measured_vector << stateEstimator->xvEst.aver_vel, stateEstimator->xvEst.b_acc;
    		
    //卡尔曼滤波器更新函数
    vaEstimateKF.update();
    //vaEstimateKF.debugPrint("update after");

    stateEstimator->xvEst.v_filter = stateEstimator->xvEst.aver_vel;
    stateEstimator->xvEst.x_filter += vaEstimateKF.x(0)*dt;
    // stateEstimator->xvEst.v_filter = vaEstimateKF.x(0);
    // stateEstimator->xvEst.x_filter += vaEstimateKF.x(0)*dt;        
}
