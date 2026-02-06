/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       engine.c/h
  * @brief      
  *             这里是5连杆轮腿动力学库, 包含状态方程，正逆运动学， 正逆动力学等API

  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2024    chenjiangnan     1.build
  *                                             
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 征途****************************
  */
#include "engine.hpp"
#include <iostream>



/*define model state-space*/
Eigen::MatrixXf modelAMatrixHandle(6,6);
float modelAMatrixData[] = {   
         0,    1.0000,         0,         0,         0,         0,
  190.0488,         0,         0,         0,    3.6421,         0,
         0,         0,         0,    1.0000,         0,         0,
  -37.6494,         0,         0,         0,    0.0689,         0,
         0,         0,         0,         0,         0,    1.0000,
   43.3754,         0,         0,         0,   47.4354,         0,
};

Eigen::MatrixXf modelBMatrixHandle(6,2);
float modelBMatrixData[] = {  
        0,         0,
-29.9582,    9.3512,
        0,         0,
    7.2051,   -1.4754,
        0,         0,
    -1.2510,   24.3695,
};
Eigen::MatrixXf modelCMatrixHandle(6,6);
float modelCMatrixData[] = {    1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                };
Eigen::MatrixXf modelDMatrixHandle(6,2);
float modelDMatrixData[] = {        1.0, 1.0,
                                    1.0, 1.0,
                                    1.0, 1.0,
                                    1.0, 1.0,
                                    1.0, 1.0,
                                    1.0, 1.0,
                                };
Eigen::MatrixXf vJacobinMatrixHandle(2,2);
float vJacobinMatrixData[] = {    0.0, 0.0,
                                    0.0, 0.0,
                                };
Eigen::MatrixXf RMatrixHandle(2,2);
float RMatrixData[] = {     0.0, 0.0,
                                0.0, 0.0,
                            };
Eigen::MatrixXf MMatrixHandle(2,2);
float MMatrixData[] = {     0.0, 0.0,
                                0.0, 0.0,
                            };                         
EngineModel_t rmInfantry_WLR;

const EngineJointConfig_t hipJointConfig[4] = {
    {0, 0 ,0, 0, - PI/9},      //hipjoint1
    {1, 0 ,0, 0,   PI*10/9},   //hipjoint2
    {3, 1, 1, 1, - PI/9},      //hiojoint3
    {4, 1 ,1, 1,   PI*10/9}    //hipjoint4
    // {1 ,0, 1, - PI/9},      //hipjoint1
    // {0 ,1, 0, - PI/9},      //hiojoint2
    // {0 ,1, 0,   PI*10/9},   //hipjoint3
    // {1 ,0, 1,   PI*10/9}    //hipjoint4
};
const EngineJointConfig_t wheelJointConfig[2] = {
    // {1 ,1, 1, 0},       //left wheel
    // {0 ,0, 0, 0},      //right wheel

    {2, 0 ,0, 0, 0},      //left wheel
    {5, 1 ,1, 1, 0}       //right wheel
};
/**
* @brief   5连杆轮腿构型机器人模型初始化
* @note    
* @param[in]   model: 轮式5连杆构型机器人模型
*/
void Engine_ModelInit(EngineModel_t* model)
{    
    if(model == NULL){
        return;
    }
    memset(model, 0, sizeof(EngineModel_t));

    for(int i = 0; i < 2; i++)
    {
        model->link.thighLink[i].lengthOrRadius = 0.138;
        model->link.shankLink[i].lengthOrRadius = 0.262;
        model->link.hipLink[i].lengthOrRadius = 0.130;
        model->link.wheelLink[i].lengthOrRadius = 0.135/2;

        model->link.thighLink[i].mass = 0;
        model->link.shankLink[i].mass = 0;
        model->link.hipLink[i].mass = 0;
        model->link.wheelLink[i].mass = 1.2;

        model->link.thighLink[i].inertia = 0;
        model->link.shankLink[i].inertia =0;
        model->link.hipLink[i].inertia = 0;
        model->link.wheelLink[i].inertia = 0;       
    }
    modelAMatrixHandle = Eigen::Map<Eigen::Matrix<float, 6, 6, Eigen::RowMajor>>(modelAMatrixData);
    modelBMatrixHandle = Eigen::Map<Eigen::Matrix<float, 6, 2, Eigen::RowMajor>>(modelBMatrixData);
    modelCMatrixHandle = Eigen::Map<Eigen::Matrix<float, 6, 6, Eigen::RowMajor>>(modelCMatrixData);
    modelDMatrixHandle = Eigen::Map<Eigen::Matrix<float, 6, 2, Eigen::RowMajor>>(modelDMatrixData);
    vJacobinMatrixHandle = Eigen::Map<Eigen::Matrix<float, 2, 2, Eigen::RowMajor>>(vJacobinMatrixData);
    RMatrixHandle = Eigen::Map<Eigen::Matrix<float, 2, 2, Eigen::RowMajor>>(RMatrixData);
    MMatrixHandle = Eigen::Map<Eigen::Matrix<float, 2, 2, Eigen::RowMajor>>(MMatrixData);
}


/**
* @brief   基于VMC的5连杆正运动学，输入5连杆笛卡尔坐标系下的[phiA,phiE] -> 输出5连杆极坐标系下的[phi0, L0]
* @note    1.正向运动学默认更新model里的fiveLink_jointFrame
*          2.const 修饰的形参为正运动学输入, 非const 修饰的形参为正运动学输出
* @param[out]   model: 轮式5连杆构型机器人模型
* @param[in]    phi1: 正运动学输入，长度为二的数组(index:0->left, index:1->right)，front髋关节电机力矩
* @param[in]    phi4: 正运动学输入，长度为二的数组(index:0->left, index:1->right)，behind髋关节电机力矩
* @param[out]   phi0: 正运动学输出，长度为二的数组(index:0->left, index:1->right)，fiveLink_jointFrame极坐标系下对应为phi0
* @param[out]   L0:   正运动学输出，长度为二的数组(index:0->left, index:1->right)，fiveLInk_jointFrame极坐标系下对应为L0
* @param[in]    ifUpdate: 布尔值，ifUpdate为True则更新fiveLink_jointFrame
*/
void Engine_ForwardKinematics(
    EngineModel_t* model, 
    const float phi1[2], 
    const float phi4[2], 
    float phi0[2], 
    float L0[2],
    EngineBool_t ifUpdate
)
{
    if(model == NULL || phi1 == NULL || phi4 == NULL || phi0 == NULL || L0 == NULL){
        return;
    }

    for(int i = 0; i < 2; i++){

        // step1:
        float x_b, y_b, x_c, y_c, x_d, y_d;
        float L1, L2, L3;
        L3 = model->link.hipLink[i].lengthOrRadius;
        L1 = model->link.thighLink[i].lengthOrRadius;
        L2 = model->link.shankLink[i].lengthOrRadius;

        x_b = L3/2 + L1*std::cos(phi1[i]);
        y_b = 0 + L1*std::sin(phi1[i]);

        x_d = -L3/2 + L1*std::cos(phi4[i]);
        y_d = 0 + L1*std::sin(phi4[i]);
        // step2:
        float A0 = -2*(x_b - x_d)*L2;
        float B0 = -2*(y_b - y_d)*L2;
        float C0 = (x_b - x_d)*(x_b - x_d) + (y_b - y_d)*(y_b - y_d);
        // step3:
        float sqrt_out = std::sqrt(A0*A0+B0*B0-C0*C0);

        float y = B0 - sqrt_out;
        float x = A0 + C0;
        float result = std::atan2(y, x);
        
        float phi2 = 2*result;
        if(phi2 < 0){
            phi2 += 2 * PI;
        }
        else if(phi2 >= 2*PI){
            phi2 -= 2 * PI;
        }
        // step4
        x_c = L1*std::cos(phi1[i]) + L2*std::cos(phi2) + L3/2;
        y_c = L1*std::sin(phi1[i]) + L2*std::sin(phi2);
        // step5
        L0[i] = std::sqrt(x_c*x_c + y_c*y_c);
        phi0[i] = std::atan2(y_c, x_c);
        // step6
        float phi3 = std::atan2(y_c-y_d, x_c-x_d);

        if(!ifUpdate){continue;}
        model->frame.fiveLink_jointFrame[i].L0 = L0[i];
        model->frame.fiveLink_jointFrame[i].phi[0] = phi0[i];
        model->frame.fiveLink_jointFrame[i].phi[1] = phi1[i]; 
        model->frame.fiveLink_jointFrame[i].phi[2] = phi2;
        model->frame.fiveLink_jointFrame[i].phi[3] = phi3;
        model->frame.fiveLink_jointFrame[i].phi[4] = phi4[i];

        // model->frame.vmc_jointFrame[i].last_theta = model->frame.vmc_jointFrame[i].theta;
        // model->frame.vmc_jointFrame[i].last_alpha = model->frame.vmc_jointFrame[i].alpha;
        model->frame.vmc_jointFrame[i].theta = phi0[i] - model->frame.body_worldFrame.rpy[1] - PI/2;
        model->frame.vmc_jointFrame[i].alpha = phi0[i] - PI/2;
        // update history data
        for(int j = 1; j < HISTORY_NUMS; j++){
            model->frame.vmc_jointFrame[i].theta_arr[HISTORY_NUMS-j] = model->frame.vmc_jointFrame[i].theta_arr[HISTORY_NUMS-1-j];
            model->frame.vmc_jointFrame[i].alpha_arr[HISTORY_NUMS-j] = model->frame.vmc_jointFrame[i].alpha_arr[HISTORY_NUMS-1-j];
            model->frame.fiveLink_jointFrame[i].L0_arr[HISTORY_NUMS-j] = model->frame.fiveLink_jointFrame[i].L0_arr[HISTORY_NUMS-1-j];
        }
        model->frame.vmc_jointFrame[i].theta_arr[0] = model->frame.vmc_jointFrame[i].theta;
        model->frame.vmc_jointFrame[i].alpha_arr[0] = model->frame.vmc_jointFrame[i].alpha;
        model->frame.fiveLink_jointFrame[i].L0_arr[0] = L0[i];
    }
} 


/**
* @brief   基于VMC的5连杆逆运动学，输入5连杆极坐标系下的[phi0, L0] -> 输出5连杆笛卡尔坐标系下的[phiA,phiE]
* @note    1.逆向运动学默认不更新model里的fiveLink_jointFrame
*          2.const 修饰的形参为逆运动学输入, 非const 修饰的形参为逆运动学输出
* @param[out]   model:轮式5连杆构型机器人模型
* @param[in]   phi0:逆运动学输入，fiveLink_jointFrame极坐标系下对应phi0
* @param[in]   L0:逆运动学输入，fiveLink_jointFrame极坐标系下对应为L0
* @param[out]   fiveLink_jointFrame: 5连杆关节坐标系，逆运动学副产物，可以为NULL
* @param[out]   phi1:逆运动学输出, fiveLink_jointFrame笛卡尔坐标系下对应为phiA
* @param[out]   phi4:逆运动学输出, fiveLInk_jointFrame笛卡尔坐标系下对应为phiE
*/
void Engine_InverseKinematics(
    EngineModel_t* model,
    const float phi0[2],
    const float L0[2],
    float phi1[2],
    float phi4[2]
)
{
    if(model == NULL || phi1 == NULL || phi4 == NULL || phi0 == NULL || L0 == NULL){
        return;
    }

    for(int i = 0; i < 2; i++){
        //step0:
        float L3 = model->link.hipLink[i].lengthOrRadius;
        float L1 = model->link.thighLink[i].lengthOrRadius;
        float L2 = model->link.shankLink[i].lengthOrRadius;
        // step1:
        float LB = std::sqrt(L0[i]*L0[i] + (L3/2)*(L3/2) - L0[i]*L3*std::cos(phi0[i])); 
        float LD = std::sqrt(L0[i]*L0[i] + (L3/2)*(L3/2) - L0[i]*L3*std::cos(PI - phi0[i]));
        // arm_sqrt_f32(L0[i]*L0[i] + (L3/2)*(L3/2) - L0[i]*L3*std::cos(phi0[i]), &LB);
        // arm_sqrt_f32(L0[i]*L0[i] + (L3/2)*(L3/2) - L0[i]*L3*std::cos(PI - phi0[i]), &LD);
        // step2:
        float phiA = std::acos( ((L3/2)*(L3/2) + LB*LB - L0[i]*L0[i])/(L3*LB)) + std::acos( (LB*LB + L1*L1 - L2*L2)/(2*LB*L1) ); 
        float phiE = std::acos( ((L3/2)*(L3/2) + LD*LD - L0[i]*L0[i])/(L3*LD)) + std::acos( (LD*LD + L1*L1 - L2*L2)/(2*LD*L1) );
        // step4:
        phi1[i] = PI - phiA;
        phi4[i] = phiE;
    }  
}


/**
* @brief   基于VMC的5连杆正动力学，输入5连杆笛卡尔坐标系下的[torqueA,torqueE] -> 输出5连杆极坐标系下的[Tp, F]
* @note    1.正向动力学默认更新model里的fiveLink_forceFrame
*          2.const 修饰的形参为正动力学输入, 非const 修饰的形参为正动力学输出
* @param[out]   model: 轮式5连杆构型机器人模型
* @param[in]   torqueA: 正动力学输入，长度为二的数组(index:0->left, index:1->right), 髋关节电机力矩, 对应5连杆坐标系下的A
* @param[in]   torqueE: 正动力学输入，长度为二的数组(index:0->left, index:1->right)，髋关节电机力矩, 对应5连杆坐标系下的E
* @param[out]   Tp:  正动力学输出，长度为二的数组(index:0->left, index:1->right)，绕机体转轴的虚拟力矩
* @param[out]   F:   正动力学输出，长度为二的数组(index:0->left, index:1->right), 沿杆长方向的虚拟力
* @param[in]   ifUpdate: 布尔值，ifUpdate为True则更新fiveLink_forceFrame
*/
void Engine_ForwardDynamics(
    EngineModel_t* model, 
    const float torqueA[2], 
    const float torqueE[2], 
    float Tp[2], 
    float F[2],
    EngineBool_t ifUpdate
){
    if(model == NULL || Tp == NULL || F == NULL || torqueA == NULL || torqueE == NULL){
        return;
    }

    for(int i = 0; i < 2; i++){
        //step0:get value
        // float L3 = model->link.hipLink[i].lengthOrRadius;
        float L1 = model->link.thighLink[i].lengthOrRadius;
        // float L2 = model->link.shankLink[i].lengthOrRadius;
        float L0 =   model->frame.fiveLink_jointFrame[i].L0;
        float phi0 = model->frame.fiveLink_jointFrame[i].phi[0];
        float phi1 = model->frame.fiveLink_jointFrame[i].phi[1];
        float phi2 = model->frame.fiveLink_jointFrame[i].phi[2];
        float phi3 = model->frame.fiveLink_jointFrame[i].phi[3];
        float phi4 = model->frame.fiveLink_jointFrame[i].phi[4];
        // step2:calculate velocity jacobin matrix and it transpose
        vJacobinMatrixHandle(0,0) = -L1*std::sin(phi1 - phi2)*std::sin(phi3)/std::sin(phi3 - phi2);
        vJacobinMatrixHandle(1,0) =  L1*std::sin(phi1 - phi2)*std::cos(phi3)/std::sin(phi3 - phi2);
        vJacobinMatrixHandle(0,1) = -L1*std::sin(phi3 - phi4)*std::sin(phi2)/std::sin(phi3 - phi2);
        vJacobinMatrixHandle(1,1) =  L1*std::sin(phi3 - phi4)*std::cos(phi2)/std::sin(phi3 - phi2);
        Eigen::MatrixXf vJT_Matrixhandle(2,2);
        float vJT_MatrixData[] = {0, 0,
                                    0, 0};
        vJT_Matrixhandle = Eigen::Map<Eigen::Matrix<float, 2, 2, Eigen::RowMajor>>(vJT_MatrixData);
        vJT_Matrixhandle = vJacobinMatrixHandle.transpose();
        // step3:calculate R matrix
        RMatrixHandle(0,0) =  std::cos(phi0 - PI/2);
        RMatrixHandle(1,0) =  std::sin(phi0 - PI/2);
        RMatrixHandle(0,1) = -std::sin(phi0 - PI/2);
        RMatrixHandle(1,1) =  std::cos(phi0 - PI/2);
        // step4:calculate M matrix
        MMatrixHandle(0,0) =  -1/L0;
        MMatrixHandle(1,0) =  0;
        MMatrixHandle(0,1) =  0;
        MMatrixHandle(1,1) =  1;
        // step5:define F matrix, F = [Tp;F]
        Eigen::MatrixXf TMatrixHandle;
        float TMatrixData[] = {torqueA[i], torqueE[i]};
        TMatrixHandle = Eigen::Map<Eigen::Matrix<float, 2, 1>>(TMatrixData);

        Eigen::MatrixXf out1MatrixHandle;
        Eigen::MatrixXf out2MatrixHandle;
        Eigen::MatrixXf out3MatrixHandle;
        Eigen::MatrixXf out4MatrixHandle;
        float out1MatrixData[2*2] = {0};
        float out2MatrixData[2*2] = {0};
        float out3MatrixData[2*2] = {0};        
        float out4MatrixData[2*1] = {0};
        out1MatrixHandle = Eigen::Map<Eigen::Matrix<float, 2, 2, Eigen::RowMajor>>(out1MatrixData);
        out2MatrixHandle = Eigen::Map<Eigen::Matrix<float, 2, 2, Eigen::RowMajor>>(out2MatrixData);
        out3MatrixHandle = Eigen::Map<Eigen::Matrix<float, 2, 2, Eigen::RowMajor>>(out3MatrixData);
        out4MatrixHandle = Eigen::Map<Eigen::Matrix<float, 2, 1>>(out4MatrixData);        
        // step6:[TA;TE] = transpose(velocity_jacobin)*R*M*F
        out1MatrixHandle = RMatrixHandle * MMatrixHandle;
        out2MatrixHandle = vJT_Matrixhandle * out1MatrixHandle;
        out3MatrixHandle = out2MatrixHandle.inverse();
        out4MatrixHandle = out3MatrixHandle * TMatrixHandle;
        
        Tp[i] = out4MatrixHandle(0);
        F[i] = out4MatrixHandle(1); 

        if(!ifUpdate){continue;}
        model->frame.fiveLink_forceFrame[i].Fx = out2MatrixHandle(0);
        model->frame.fiveLink_forceFrame[i].Fy = out2MatrixHandle(1);
        model->frame.vmc_forceFrame[i].F = F[i];
        model->frame.vmc_forceFrame[i].Tp = Tp[i];
    }       
}


/**
* @brief   基于VMC的5连杆逆动力学，输入5连杆极坐标系下的[Tp,F] -> 输出5连杆笛卡尔标系下的[torqueA, torqueE]
* @note    1.逆向动力学默认不更新model里的vmc_forceFrame
*          2.const 修饰的形参为逆动力学输入, 非const 修饰的形参为逆动力学输出
* @param[out]   model: 轮式5连杆构型机器人模型
* @param[in]    Tp:  正动力学输入，长度为二的数组(index:0->left, index:1->right)，绕机体转轴的虚拟力矩
* @param[in]    F:   正动力学输入，长度为二的数组(index:0->left, index:1->right), 沿杆长方向的虚拟力
* @param[out]   torqueA: 正动力学输入，长度为二的数组(index:0->left, index:1->right), 髋关节电机力矩, 对应5连杆坐标系下的A
* @param[out]   torqueE: 正动力学输入，长度为二的数组(index:0->left, index:1->right)，髋关节电机力矩, 对应5连杆坐标系下的E
* @param[in]    ifUpdate: 布尔值，ifUpdate为True则更新fiveLink_forceFrame以及vmc_forceFrame
*/
void Engine_InverseDynamics(
    EngineModel_t* model, 
    const float Tp[2], 
    const float F[2],
    float torqueA[2], 
    float torqueE[2],
    EngineBool_t ifUpdate
)
{
    if(model == NULL || Tp == NULL || F == NULL || torqueA == NULL || torqueE == NULL){
        return;
    }

    for(int i = 0; i < 2; i++){
        // step0:get value
        // float L3 = model->link.hipLink[i].lengthOrRadius;
        float L1 = model->link.thighLink[i].lengthOrRadius;
        // float L2 = model->link.shankLink[i].lengthOrRadius;
        float L0 =   model->frame.fiveLink_jointFrame[i].L0;
        float phi0 = model->frame.fiveLink_jointFrame[i].phi[0];
        float phi1 = model->frame.fiveLink_jointFrame[i].phi[1];
        float phi2 = model->frame.fiveLink_jointFrame[i].phi[2];
        float phi3 = model->frame.fiveLink_jointFrame[i].phi[3];
        float phi4 = model->frame.fiveLink_jointFrame[i].phi[4];
        // step2:calculate velocity jacobin matrix and it transpose
        vJacobinMatrixHandle(0,0) = -L1*std::sin(phi1 - phi2)*std::sin(phi3)/std::sin(phi3 - phi2);
        vJacobinMatrixHandle(1,0) =  L1*std::sin(phi1 - phi2)*std::cos(phi3)/std::sin(phi3 - phi2);
        vJacobinMatrixHandle(0,1) = -L1*std::sin(phi3 - phi4)*std::sin(phi2)/std::sin(phi3 - phi2);
        vJacobinMatrixHandle(1,1) =  L1*std::sin(phi3 - phi4)*std::cos(phi2)/std::sin(phi3 - phi2);
        Eigen::MatrixXf vJT_Matrixhandle(2,2);
        float vJT_MatrixData[] = {0, 0,
                                    0, 0};
        vJT_Matrixhandle = Eigen::Map<Eigen::Matrix<float, 2, 2, Eigen::RowMajor>>(vJT_MatrixData);  
        vJT_Matrixhandle = vJacobinMatrixHandle.transpose();                           
        // step3:calculate R matrix
        RMatrixHandle(0,0) =  std::cos(phi0 - PI/2);
        RMatrixHandle(1,0) =  std::sin(phi0 - PI/2);
        RMatrixHandle(0,1) = -std::sin(phi0 - PI/2);
        RMatrixHandle(1,1) =  std::cos(phi0 - PI/2);
        // step4:calculate M matrix
        MMatrixHandle(0,0) =  -1/L0;
        MMatrixHandle(1,0) =  0;
        MMatrixHandle(0,1) =  0;
        MMatrixHandle(1,1) =  1;
        // step5:define F matrix, F = [Tp;F]
        Eigen::MatrixXf FMatrixHandle(2,1);
        float FMatrixData[] = {Tp[i], F[i]};
        FMatrixHandle = Eigen::Map<Eigen::Matrix<float, 2, 1>>(FMatrixData);  

        Eigen::MatrixXf out1MatrixHandle(2,1);
        Eigen::MatrixXf out2MatrixHandle(2,1);
        Eigen::MatrixXf out3MatrixHandle(2,1);
        float out1MatrixData[] = {0,0};
        float out2MatrixData[] = {0,0};
        float out3MatrixData[] = {0,0};
        out1MatrixHandle = Eigen::Map<Eigen::Matrix<float, 2, 1>>(out1MatrixData);
        out2MatrixHandle = Eigen::Map<Eigen::Matrix<float, 2, 1>>(out2MatrixData);        
        out3MatrixHandle = Eigen::Map<Eigen::Matrix<float, 2, 1>>(out3MatrixData);
        // step6:[TA;TE] = transpose(velocity_jacobin)*R*M*F
        out1MatrixHandle = MMatrixHandle * FMatrixHandle;
        out2MatrixHandle = RMatrixHandle * out1MatrixHandle;
        out3MatrixHandle = vJT_Matrixhandle * out2MatrixHandle;
        
        torqueA[i] = out3MatrixHandle(0);
        torqueE[i] = out3MatrixHandle(1); 

        if(!ifUpdate){continue;}
        model->frame.fiveLink_forceFrame[i].Fx = out2MatrixHandle(0);
        model->frame.fiveLink_forceFrame[i].Fy = out2MatrixHandle(1);
        model->frame.vmc_forceFrame[i].F = FMatrixHandle(1);
        model->frame.vmc_forceFrame[i].Tp = FMatrixHandle(0);
    }     
}


/**
* @brief   轮式5连杆构型机器人状态空间方程运算
*          X_dot = A*X + B*U
* @note    1.x u x_dot 均为1维数组
* @param[in]    x:   系统状态矩阵 维度：6x1 [theta theta_dot x x_dot phi phi_dot]
* @param[in]    u:   系统输入矩阵 维度: 2x1 [Tp T]
* @param[out]   x_dot:系统状态微分矩阵 维度:6x1 
*/
void Engine_SystemSpaceCalculate(
    float x[2][6],
    float u[2][2],
    float x_dot[2][6]
){
    if(x == NULL || u == NULL || x_dot == NULL){
        return;
    }
    for(int i = 0; i < 2; i++){
        Eigen::MatrixXf XMatrixHandle(6,1);
        float XMatrixData[6*1] = {0};
        Eigen::MatrixXf UMatrixHandle(2,1);
        float UMatrixData[2*1] = {0};
        memcpy(XMatrixData, &x[i], (XMatrixHandle.cols()*XMatrixHandle.rows())*sizeof(float));
        memcpy(UMatrixData, &u[i], (UMatrixHandle.cols()*UMatrixHandle.rows())*sizeof(float));
    
        XMatrixHandle = Eigen::Map<Eigen::Matrix<float, 6, 1>>(XMatrixData);
        UMatrixHandle = Eigen::Map<Eigen::Matrix<float, 2, 1>>(UMatrixData);
    
        Eigen::MatrixXf AXMatrixHandle(6,1);
        float AXMatrixData[6*1] = {0}; 
        Eigen::MatrixXf BUMatrixHandle(6,1);
        float BUMatrixData[6*1] = {0}; 
        Eigen::MatrixXf XdotMatrixHandle(6,1);
        float XdotMatrixData[6*1] = {0}; 
    
        AXMatrixHandle = Eigen::Map<Eigen::Matrix<float, 6, 1>>(AXMatrixData);
        BUMatrixHandle = Eigen::Map<Eigen::Matrix<float, 6, 1>>(BUMatrixData);
        XdotMatrixHandle = Eigen::Map<Eigen::Matrix<float, 6, 1>>(XdotMatrixData);
        /*x_dot = A*x + B*u*/
        AXMatrixHandle = modelAMatrixHandle * XMatrixHandle;
        BUMatrixHandle = modelBMatrixHandle * UMatrixHandle;
        XdotMatrixHandle = AXMatrixHandle + BUMatrixHandle;
    
        memcpy(&x_dot[i], XdotMatrixData, (XdotMatrixHandle.cols()*XdotMatrixHandle.rows())*sizeof(float));
    }

}

// void Engine_FnCalculate(
// EngineModel_t* model,
// float  steptime,
// float* Fn
// ){
//     if(model == NULL || Fn == NULL){
//         return;
//     }
//     static float L0Array[2][3 + 1] = {0};
//     static float thetaArray[2][3 + 1] = {0};
//     /*step1:torqueA:torqueE map to Tp:Fn*/
//     float torqueA[2] = {model->joint.hipJoint[0].t, model->joint.hipJoint[1].t};
//     float torqueE[2] = {model->joint.hipJoint[3].t, model->joint.hipJoint[2].t};
//     float Tp[2] = {0};
//     float F[2] = {0};
//     Engine_ForwardDynamics(model, torqueA, torqueE, Tp, F, 0);
//     for(int i = 0; i < 2; i++){
//         /*step2:calculate the wheel acc*/
//         if(L0Array[i][0] == 0 || thetaArray[i][0] == 0){
//             L0Array[i][0] = 1;
//             thetaArray[i][0] = 1;
//             L0Array[i][1] = L0Array[i][2] = L0Array[i][3] = model->frame.fiveLink_jointFrame[i].L0;
//             thetaArray[i][1] = thetaArray[i][2] = thetaArray[i][3] = model->frame.vmc_jointFrame[i].theta;
//         }
//         for(int j = 0; j < 2; j++){
//             L0Array[i][3-j] = L0Array[i][3-j -1];
//             thetaArray[i][3-j] = thetaArray[i][3-j -1];
//         }
//         L0Array[i][1] = model->frame.fiveLink_jointFrame[i].L0;
//         thetaArray[i][1] =  model->frame.vmc_jointFrame[i].theta;

//         float zmddot = model->frame.body_worldFrame.a[2];
//         float L0ddot = ( (L0Array[i][1] - L0Array[i][2])/steptime - (L0Array[i][2] - L0Array[i][3])/steptime ) / steptime;
//         float L0dot = (L0Array[i][1] - L0Array[i][2])/steptime;
//         float thetaddot =  ( (thetaArray[i][1] - thetaArray[i][2])/steptime - (thetaArray[i][2] - thetaArray[i][3])/steptime ) / steptime;
//         float thetadot = (thetaArray[i][1] - thetaArray[i][2])/steptime;
//         float zwddot =  zmddot - L0ddot*std::cos(thetaArray[i][1]) + 2*L0dot*thetadot*std::sin(thetaArray[i][1]) + L0Array[i][1]*( thetaddot*std::sin(thetaArray[i][1]) + thetadot*thetadot*std::cos(thetaArray[i][1]) );

//         /*step3:P = Fcos(theta) + Tp*sin(theta)/L0*/
//         float P = F[i]*std::cos(thetaArray[i][1]) + Tp[i]*std::sin(thetaArray[i][1])/L0Array[i][1];
//         /*step4:fn = mw*zwddot + p + mw*g*/
//         Fn[i] = P + model->link.wheelLink[i].mass* (9.8 + zwddot); 
//     }
// }

void PrintEngineModelInfo(const EngineModel_t* model) {
    std::cout << "\n========== EngineModel Debug Info ==========" << std::endl;
    
    // 打印身体姿态
    std::cout << "\n--- Body World Frame ---" << std::endl;
    std::cout << "Roll:  " << std::setw(10) << std::fixed << std::setprecision(4) << model->frame.body_worldFrame.rpy[0] 
              << " rad, Pitch: " << std::setw(10) << model->frame.body_worldFrame.rpy[1]
              << " rad, Yaw: " << std::setw(10) << model->frame.body_worldFrame.rpy[2] << " rad" << std::endl;
    std::cout << "Angular Vel: [" 
              << std::setw(8) << model->frame.body_worldFrame.w[0] << ", "
              << std::setw(8) << model->frame.body_worldFrame.w[1] << ", "
              << std::setw(8) << model->frame.body_worldFrame.w[2] << "] rad/s" << std::endl;
    std::cout << "Acceleration: [" 
              << std::setw(8) << model->frame.body_worldFrame.a[0] << ", "
              << std::setw(8) << model->frame.body_worldFrame.a[1] << ", "
              << std::setw(8) << model->frame.body_worldFrame.a[2] << "] m/s²" << std::endl;
    
    // 打印关节状态（按左右侧）
    for (int side = 0; side < 2; side++) {
        std::cout << "\n--- " << (side == 0 ? "Left Side" : "Right Side") << " ---" << std::endl;
        
        // 5连杆关节帧
        std::cout << "FiveLink Frame:" << std::endl;
        std::cout << "  L0:  " << std::setw(10) << model->frame.fiveLink_jointFrame[side].L0 << " m" << std::endl;
        std::cout << "  Phi: [";
        for (int i = 0; i < 5; i++) {
            std::cout << std::setw(8) << model->frame.fiveLink_jointFrame[side].phi[i];
            if (i < 4) std::cout << ", ";
        }
        std::cout << "] rad" << std::endl;
        
        // VMC关节帧
        std::cout << "VMC Joint Frame:" << std::endl;
        std::cout << "  Theta: " << std::setw(10) << model->frame.vmc_jointFrame[side].theta 
                  << " rad, APhi: " << std::setw(10) << model->frame.vmc_jointFrame[side].alpha << " rad" << std::endl;
        
        // VMC力帧
        std::cout << "VMC Force Frame:" << std::endl;
        std::cout << "  Tp: " << std::setw(10) << model->frame.vmc_forceFrame[side].Tp 
                  << " N·m, F: " << std::setw(10) << model->frame.vmc_forceFrame[side].F << " N" << std::endl;
        
        // 髋关节（4个） - 修改为左: 0,3, 右: 1,2
        std::cout << "Hip Joints:" << std::endl;
        if (side == 0) {  // 左: 0,3
            std::cout << "  Joint 0: q=" << std::setw(8) << model->joint.hipJoint[0].q 
                      << " rad, w=" << std::setw(8) << model->joint.hipJoint[0].w 
                      << " rad/s, t=" << std::setw(8) << model->joint.hipJoint[0].t << " N·m" << std::endl;
            std::cout << "  Joint 3: q=" << std::setw(8) << model->joint.hipJoint[3].q 
                      << " rad, w=" << std::setw(8) << model->joint.hipJoint[3].w 
                      << " rad/s, t=" << std::setw(8) << model->joint.hipJoint[3].t << " N·m" << std::endl;
        } else {  // 右: 1,2
            std::cout << "  Joint 1: q=" << std::setw(8) << model->joint.hipJoint[1].q 
                      << " rad, w=" << std::setw(8) << model->joint.hipJoint[1].w 
                      << " rad/s, t=" << std::setw(8) << model->joint.hipJoint[1].t << " N·m" << std::endl;
            std::cout << "  Joint 2: q=" << std::setw(8) << model->joint.hipJoint[2].q 
                      << " rad, w=" << std::setw(8) << model->joint.hipJoint[2].w 
                      << " rad/s, t=" << std::setw(8) << model->joint.hipJoint[2].t << " N·m" << std::endl;
        }
        
        // 轮关节
        std::cout << "Wheel Joint " << side << ":" << std::endl;
        std::cout << "  q=" << std::setw(8) << model->joint.wheelJoint[side].q 
                  << " rad, w=" << std::setw(8) << model->joint.wheelJoint[side].w 
                  << " rad/s, t=" << std::setw(8) << model->joint.wheelJoint[side].t << " N·m" << std::endl;
    }
    std::cout << "\n============================================\n" << std::endl;
}