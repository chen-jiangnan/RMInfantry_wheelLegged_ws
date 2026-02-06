/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       engine.c/h
  * @brief      
  *             这里是5连杆轮腿动力学库, 包含正逆运动学， 正逆动力学等API

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
#ifndef __ENGINE_H
#define __ENGINE_H

#include <cstdint>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <iomanip>

#define HISTORY_NUMS 10
const float PI = 3.14159265358979323846;

/*define model state-space*/
extern Eigen::MatrixXf modelAMatrixHandle;
extern Eigen::MatrixXf modelBMatrixHandle;
  
typedef struct{
    float q;    //关节角位置，电机反馈值    单位:rad
    float w;    //关节角速度, 电机反馈值    单位:rad/s
    float t;    //关节角力矩, 电机反馈值    单位:n*m
}EngineJoint_t;// 通过电机反馈进行更新

typedef struct{
    float mass;             //质量描述              单位:kg
    float lengthOrRadius;   //长度或者半径描述       单位:m
    float inertia;          //转动惯量描述(绕质心)   单位:kg*m^2   
}EngineLink_t;// 初始化设置

typedef struct{
    float mass;             //质量描述                        单位:kg
    float l;                //虚拟连杆的转轴到机体质心的距离      单位:m
    float inertia;          //转动惯量描述                     单位:kg*m^2
}EngineBodyLink_t;// 初始化设置

typedef struct{
    float mass;             //质量描述                     单位:kg
    float L;                //车轮轴心到虚拟连杆质心的距离     单位:m
    float LM;               //虚拟杆质心到虚拟杆转轴的距离     单位:m
    float inertia;          //转动惯量描述                    单位:kg*m^2
}EngineVirtualLink_t;// 初始化设置

typedef struct{
    float L0;               //虚拟杆长                                                                     单位:m
    float phi[5];           //5连杆各杆在该坐标系下与x轴的角度，其中index:0为虚拟杆长,index:1~4从A点按顺时针编号    单位:rad
    float L0_arr[HISTORY_NUMS];       //储存历史数据
}EngineFiveLinkJointFrame_t;// 通过电机反馈进行更新

typedef struct{
    float theta;            //虚拟杆在世界坐标系下与竖直方向的夹角      单位:rad
    float alpha;             //虚拟杆在机体坐标系下与竖直方向的夹角      单位:rad
    float theta_arr[HISTORY_NUMS];    //储存历史数据
    float alpha_arr[HISTORY_NUMS];     //储存历史数据
}EngineVMCJointFrame_t;// 通过电机反馈进行更新

typedef struct{
    float Tp;               //虚拟连杆坐标系下绕机体转轴旋转的力矩      单位:n*m
    float F;                //虚拟连杆坐标系下沿杆长方向维持杆长的力    单位:n
}EngineVMCForceFrame_t;

typedef struct{
    float rpy[3];           //机体在世界坐标系下的翻滚角roll/俯仰角pitch/航向角yaw        单位:rad
    float w[3];
    float a[3];
}EngineBody_WorldFrame_t;

typedef struct{
    float Fx;       //5连杆坐标系下，踝关节处沿x轴方向的力          单位:n
    float Fy;       //5连杆坐标系下, 踝关节处沿y轴方向的力          单位:n
}EngineFiveLinkForceFrame_t;

typedef uint8_t EngineBool_t;

typedef struct{
    /*define link*/
    struct{
        EngineBodyLink_t bodyLink;          //机体
        EngineLink_t hipLink[2];            //髋关节之间
        EngineLink_t thighLink[2];          //大腿
        EngineLink_t shankLink[2];          //小腿
        EngineLink_t wheelLink[2];          //轮
        EngineVirtualLink_t vitrualLink[2]; //摆杆
    }link;

    /*define joint*/
    struct{
        EngineJoint_t hipJoint[4];    //关节电机(髋关节)
        EngineJoint_t wheelJoint[2];     //驱动轮
    }joint;

    /*define frame*/
    struct {
        EngineFiveLinkJointFrame_t fiveLink_jointFrame[2];
        EngineFiveLinkForceFrame_t fiveLink_forceFrame[2];
        EngineVMCJointFrame_t   vmc_jointFrame[2];
        EngineVMCForceFrame_t   vmc_forceFrame[2];
        EngineBody_WorldFrame_t body_worldFrame;
    }frame;
    
}EngineModel_t;

extern EngineModel_t rmInfantry_WLR;

typedef struct{
    uint8_t index;
    uint8_t ifInvertPos;
    uint8_t ifInvertVel;
    uint8_t ifInvertTorque;
    float posOffset;
}EngineJointConfig_t;
extern const EngineJointConfig_t hipJointConfig[4];
extern const EngineJointConfig_t wheelJointConfig[2];
/**
* @brief   5连杆轮腿构型机器人模型初始化
* @note    
* @param[in]   model: 轮式5连杆构型机器人模型
*/
void Engine_ModelInit(EngineModel_t* model);


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
);


/**
* @brief   基于VMC的5连杆逆运动学，输入5连杆极坐标系下的[phi0, L0] -> 输出5连杆笛卡尔坐标系下的[phiA,phiE]
* @note    1.逆向运动学默认不更新model里的fiveLink_jointFrame
*          2.const 修饰的形参为逆运动学输入, 非const 修饰的形参为逆运动学输出
* @param[out]   model:轮式5连杆构型机器人模型
* @param[in]    phi0:逆运动学输入，fiveLink_jointFrame极坐标系下对应phi0
* @param[in]    L0:逆运动学输入，fiveLink_jointFrame极坐标系下对应为L0
* @param[out]   phi1:逆运动学输出, fiveLink_jointFrame笛卡尔坐标系下对应为phiA
* @param[out]   phi4:逆运动学输出, fiveLInk_jointFrame笛卡尔坐标系下对应为phiE
*/
void Engine_InverseKinematics(
    EngineModel_t* model,
    const float phi0[2],
    const float L0[2],
    float phi1[2],
    float phi4[2]
);

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
);


/**
* @brief   基于VMC的5连杆逆动力学，输入5连杆极坐标系下的[Tp,F] -> 输出5连杆笛卡尔标系下的[torqueA, torqueE]
* @note    1.逆向动力学默认不更新model里的vmc_forceFrame
*          2.const 修饰的形参为逆动力学输入, 非const 修饰的形参为逆动力学输出
* @param[out]   model: 轮式5连杆构型机器人模型
* @param[in]   Tp:  正动力学输入，长度为二的数组(index:0->left, index:1->right)，绕机体转轴的虚拟力矩
* @param[in]   F:   正动力学输入，长度为二的数组(index:0->left, index:1->right), 沿杆长方向的虚拟力
* @param[out]   torqueA: 正动力学输入，长度为二的数组(index:0->left, index:1->right), 髋关节电机力矩, 对应5连杆坐标系下的A
* @param[out]   torqueE: 正动力学输入，长度为二的数组(index:0->left, index:1->right)，髋关节电机力矩, 对应5连杆坐标系下的E
*/
void Engine_InverseDynamics(
    EngineModel_t* model, 
    const float Tp[2], 
    const float F[2],
    float torqueA[2], 
    float torqueE[2],
    EngineBool_t ifUpdate
);


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
);

void PrintEngineModelInfo(const EngineModel_t* model); 
#endif
  