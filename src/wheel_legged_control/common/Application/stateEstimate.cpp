#include "stateEstimate.hpp"



void stateEstimatorInit(StateEstimatorHandle_t* stateEstimator){
    memset(stateEstimator, 0, sizeof(StateEstimatorHandle_t));
}

void FnEstimateUpdate(StateEstimatorHandle_t* stateEstimator, 
    const float* theta, 
    const float* L0,  
    const float* F, 
    const float* Tp,
    float ddot_zb,
    float mw, 
    float dt
){
    //
    for(int i = 0; i < 2; i++){
        // update input value
        stateEstimator->FnEst[i].theta[2] = stateEstimator->FnEst[i].theta[1];
        stateEstimator->FnEst[i].theta[1] = stateEstimator->FnEst[i].theta[0];
        stateEstimator->FnEst[i].theta[0] = theta[i];
        stateEstimator->FnEst[i].L0[2] = stateEstimator->FnEst[i].L0[1];
        stateEstimator->FnEst[i].L0[1] = stateEstimator->FnEst[i].L0[0];
        stateEstimator->FnEst[i].L0[0] = L0[i];
        stateEstimator->FnEst[i].ddot_zb = ddot_zb;
        // P = Fcos(theta) + Tpsin(theta)/L0
        float P = F[i]*std::cos(theta[i]) + Tp[i]*std::sin(theta[i])/stateEstimator->FnEst[i].L0[0];
        // ddot_zw = ddot_zb - ddot_L0*cos(theta) + 2*dot_L0*dot_theta*sin(theta) + L0*ddot_theta*sin(theta) + L0*(d_theta)^2*cos(theta)
        float dot_theta = (stateEstimator->FnEst[i].theta[0] - stateEstimator->FnEst[i].theta[1])/dt;
        float ddot_theta = (stateEstimator->FnEst[i].theta[0] - 2*stateEstimator->FnEst[i].theta[1] + stateEstimator->FnEst[i].theta[2])/(dt*dt);
        float dot_L0 = (stateEstimator->FnEst[i].L0[0] - stateEstimator->FnEst[i].L0[1])/dt;
        float ddot_L0 = (stateEstimator->FnEst[i].L0[0] - 2*stateEstimator->FnEst[i].L0[1] + stateEstimator->FnEst[i].L0[2])/(dt*dt); 
        // ddot_L0 = ddot_theta = 0;
        float ddot_zw = ddot_zb - ddot_L0*std::cos(theta[i]) 
                        + 2*dot_L0*dot_theta*std::sin(theta[i]) 
                        + stateEstimator->FnEst[i].L0[0]*ddot_theta*std::sin(theta[i])
                        + stateEstimator->FnEst[i].L0[0]*(dot_theta*dot_theta)*std::cos(theta[i]);  
        float Fn = mw*ddot_zw + mw*9.8 + P;
        // update output value
        stateEstimator->FnEst[i].ddot_zw = ddot_zw;
        stateEstimator->FnEst[i].Fn = Fn;

        stateEstimator->FnEst[i].ddot_L0 = ddot_L0;
        stateEstimator->FnEst[i].dot_L0 = dot_L0;
        stateEstimator->FnEst[i].ddot_theta = ddot_theta;
        stateEstimator->FnEst[i].dot_theta = dot_theta;     
    }
}