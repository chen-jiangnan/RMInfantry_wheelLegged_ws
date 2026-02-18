/**
 * @file WheelLeggedRobot.cpp
 * @brief 五连杆轮腿机器人类实现
 * 
 * 包含完整的运动学和动力学计算过程
 */

#include "WheelLeggedRobot.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>

namespace robot {

// ==================== 构造函数 ====================

WheelLeggedRobot::WheelLeggedRobot() {
    initialize();
}

WheelLeggedRobot::WheelLeggedRobot(const Config& config) 
    : config_(config) 
{
    initialize();
}

// ==================== 配置管理 ====================

void WheelLeggedRobot::setConfig(const Config& config) {
    config_ = config;
    initialize();
}

const WheelLeggedRobot::Config& WheelLeggedRobot::getConfig() const {
    return config_;
}

// ==================== 正运动学 ====================

/**
* @brief 正运动学详细实现
* 
* 五连杆构型:
*        A ----------- E
*       /|             |\
*    l1/ |             | \l4
*     /  |l2         l3|  \
*    B   |             |   D
*     \  |             |  /
*    l2\ |             | /l3
*       \|             |/
*        C (踝关节/轮轴心)
* 
* 计算过程:
* 1. 从phi1计算B点坐标
* 2. 从phi4计算D点坐标
* 3. 计算BD距离
* 4. 用余弦定理计算angle_DBC
* 5. 计算phi2
* 6. 计算C点坐标
* 7. 计算L0和phi0
*/
void WheelLeggedRobot::forwardKinematics(
    const std::array<float, 2>& phi1,
    const std::array<float, 2>& phi4,
    std::array<float, 2>& phi0,
    std::array<float, 2>& L0,
    bool update_frame
) {
    for (int i = 0; i < 2; ++i) {
        // step1:
        float x_b, y_b, x_c, y_c, x_d, y_d;
        float L1, L2, L3;
        L3 = config_.hip_joint_distance;
        L1 = config_.thigh_link[i].lengthOrRadius;
        L2 = config_.shank_link[i].lengthOrRadius;

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

        // ========== 更新内部状态 ==========
        if (update_frame) {

            fivelink_joint_frame_[i].L0 = L0[i];
            fivelink_joint_frame_[i].phi[0] = phi0[i];  
            fivelink_joint_frame_[i].phi[1] = phi1[i];  
            fivelink_joint_frame_[i].phi[2] = phi2;     
            fivelink_joint_frame_[i].phi[3] = phi3;     
            fivelink_joint_frame_[i].phi[4] = phi4[i]; 

            vmc_joint_frame_[i].theta = phi0[i] + body_world_frame_.rpy[1] - PI/2; //
            vmc_joint_frame_[i].alpha = phi0[i] - PI/2;
            
            // 更新历史数据
            for (int j = HISTORY_NUMS - 1; j > 0; --j) {
                vmc_joint_frame_[i].theta_history[j] = vmc_joint_frame_[i].theta_history[j - 1];
                vmc_joint_frame_[i].alpha_history[j] = vmc_joint_frame_[i].alpha_history[j - 1];
                fivelink_joint_frame_[i].L0_history[j] = fivelink_joint_frame_[i].L0_history[j - 1];
            }
            vmc_joint_frame_[i].theta_history[0] = vmc_joint_frame_[i].theta;
            vmc_joint_frame_[i].alpha_history[0] = vmc_joint_frame_[i].alpha;
            fivelink_joint_frame_[i].L0_history[0] = L0[i];
        }
    }
}

// ==================== 逆运动学 ====================

/**
* @brief 逆运动学详细实现
* 
* 计算过程:
* 1. 从(phi0, L0)计算C点坐标
* 2. 计算AC距离
* 3. 用余弦定理计算angle_BAC
* 4. 计算phi1
* 5. 计算EC距离
* 6. 用余弦定理计算angle_DEC
* 7. 计算phi4
*/
void WheelLeggedRobot::inverseKinematics(
    const std::array<float, 2>& phi0,
    const std::array<float, 2>& L0,
    std::array<float, 2>& phi1,
    std::array<float, 2>& phi4
) {
    for (int i = 0; i < 2; ++i) {
        //step0:
        float L3 = config_.hip_joint_distance;
        float L1 = config_.thigh_link[i].lengthOrRadius;
        float L2 = config_.shank_link[i].lengthOrRadius;
        // step1:
        float LB = std::sqrt(L0[i]*L0[i] + (L3/2)*(L3/2) - L0[i]*L3*std::cos(phi0[i])); 
        float LD = std::sqrt(L0[i]*L0[i] + (L3/2)*(L3/2) - L0[i]*L3*std::cos(PI - phi0[i]));
        // step2:
        float phiA = std::acos( ((L3/2)*(L3/2) + LB*LB - L0[i]*L0[i])/(L3*LB)) + std::acos( (LB*LB + L1*L1 - L2*L2)/(2*LB*L1) ); 
        float phiE = std::acos( ((L3/2)*(L3/2) + LD*LD - L0[i]*L0[i])/(L3*LD)) + std::acos( (LD*LD + L1*L1 - L2*L2)/(2*LD*L1) );
        // step4:
        phi1[i] = PI - phiA;
        phi4[i] = phiE;
    } 
}

// ==================== 正动力学 ====================

/**
* @brief 正动力学详细实现 (VMC)
* 
* 使用VMC雅可比矩阵将关节空间力矩映射到虚拟空间力/力矩
* 
* tau = J^T * F_vmc
* F_vmc = (J^T)^-1 * tau
*/
void WheelLeggedRobot::forwardDynamics(
    const std::array<float, 2>& torqueA,
    const std::array<float, 2>& torqueE,
    std::array<float, 2>& Tp,
    std::array<float, 2>& F,
    bool update_frame
) {
    for (int i = 0; i < 2; ++i) {
        //step0:get value
        // float L3 = model->link.hipLink[i].lengthOrRadius;
        float L1 = config_.thigh_link[i].lengthOrRadius;
        // float L2 = model->link.shankLink[i].lengthOrRadius;
        float L0 =   fivelink_joint_frame_[i].L0;
        float phi0 = fivelink_joint_frame_[i].phi[0];
        float phi1 = fivelink_joint_frame_[i].phi[1];
        float phi2 = fivelink_joint_frame_[i].phi[2];
        float phi3 = fivelink_joint_frame_[i].phi[3];
        float phi4 = fivelink_joint_frame_[i].phi[4];
        // step2:calculate velocity jacobin matrix and it transpose
        Eigen::MatrixXf vJacobian_Matrix(2,2);
        vJacobian_Matrix(0,0) = -L1*std::sin(phi1 - phi2)*std::sin(phi3)/std::sin(phi3 - phi2);
        vJacobian_Matrix(1,0) =  L1*std::sin(phi1 - phi2)*std::cos(phi3)/std::sin(phi3 - phi2);
        vJacobian_Matrix(0,1) = -L1*std::sin(phi3 - phi4)*std::sin(phi2)/std::sin(phi3 - phi2);
        vJacobian_Matrix(1,1) =  L1*std::sin(phi3 - phi4)*std::cos(phi2)/std::sin(phi3 - phi2);
        Eigen::MatrixXf vJacobian_T_Matrix(2,2);
        vJacobian_T_Matrix = vJacobian_Matrix.transpose();
        // step3:calculate R matrix
        Eigen::MatrixXf R_Matrix(2,2);
        R_Matrix(0,0) =  std::cos(phi0 - PI/2);
        R_Matrix(1,0) =  std::sin(phi0 - PI/2);
        R_Matrix(0,1) = -std::sin(phi0 - PI/2);
        R_Matrix(1,1) =  std::cos(phi0 - PI/2);
        // step4:calculate M matrix
        Eigen::MatrixXf M_Matrix(2,2);
        M_Matrix(0,0) =  -1/L0;
        M_Matrix(1,0) =  0;
        M_Matrix(0,1) =  0;
        M_Matrix(1,1) =  1;
        // step5:define F matrix, F = [Tp;F]
        Eigen::MatrixXf T_Matrix;
        float T_MatrixData[] = {torqueA[i], torqueE[i]};
        T_Matrix = Eigen::Map<Eigen::Matrix<float, 2, 1>>(T_MatrixData);

        Eigen::MatrixXf out1_Matrix;
        Eigen::MatrixXf out2_Matrix;
        Eigen::MatrixXf out3_Matrix;
        float out1_MatrixData[2*1] = {0};
        float out2_MatrixData[2*2] = {0};     
        float out3_MatrixData[2*1] = {0};
        out1_Matrix = Eigen::Map<Eigen::Matrix<float, 2, 1>>(out1_MatrixData);
        out2_Matrix = Eigen::Map<Eigen::Matrix<float, 2, 2, Eigen::RowMajor>>(out2_MatrixData);
        out3_Matrix = Eigen::Map<Eigen::Matrix<float, 2, 1>>(out3_MatrixData);        
        // step6:[TA;TE] = transpose(velocity_jacobin)*R*M*F
        out1_Matrix = vJacobian_T_Matrix.inverse() * T_Matrix;
        out2_Matrix = R_Matrix * M_Matrix;
        out3_Matrix =  out2_Matrix.inverse() * out1_Matrix;
        
        Tp[i] = out3_Matrix(0);
        F[i] = out3_Matrix(1); 

        if(update_frame){
            fivelink_force_frame_[i].Fx = out1_Matrix(0);
            fivelink_force_frame_[i].Fy = out1_Matrix(1);
            vmc_force_frame_[i].F = F[i];
            vmc_force_frame_[i].Tp = Tp[i];
        }
    }
}


// ==================== 逆动力学 ====================

/**
* @brief 逆动力学详细实现 (VMC)
* 
* 使用VMC雅可比矩阵将虚拟空间力/力矩映射到关节空间力矩
* 
* tau = J^T * F_vmc
*/
void WheelLeggedRobot::inverseDynamics(
    const std::array<float, 2>& Tp,
    const std::array<float, 2>& F,
    std::array<float, 2>& torqueA,
    std::array<float, 2>& torqueE
) {
    for (int i = 0; i < 2; ++i) {
        //step0:get value
        // float L3 = model->link.hipLink[i].lengthOrRadius;
        float L1 = config_.thigh_link[i].lengthOrRadius;
        // float L2 = model->link.shankLink[i].lengthOrRadius;
        float L0 =   fivelink_joint_frame_[i].L0;
        float phi0 = fivelink_joint_frame_[i].phi[0];
        float phi1 = fivelink_joint_frame_[i].phi[1];
        float phi2 = fivelink_joint_frame_[i].phi[2];
        float phi3 = fivelink_joint_frame_[i].phi[3];
        float phi4 = fivelink_joint_frame_[i].phi[4];
        // step2:calculate velocity jacobin matrix and it transpose
        Eigen::MatrixXf vJacobian_Matrix(2,2);
        vJacobian_Matrix(0,0) = -L1*std::sin(phi1 - phi2)*std::sin(phi3)/std::sin(phi3 - phi2);
        vJacobian_Matrix(1,0) =  L1*std::sin(phi1 - phi2)*std::cos(phi3)/std::sin(phi3 - phi2);
        vJacobian_Matrix(0,1) = -L1*std::sin(phi3 - phi4)*std::sin(phi2)/std::sin(phi3 - phi2);
        vJacobian_Matrix(1,1) =  L1*std::sin(phi3 - phi4)*std::cos(phi2)/std::sin(phi3 - phi2);
        Eigen::MatrixXf vJacobian_T_Matrix(2,2);
        vJacobian_T_Matrix = vJacobian_Matrix.transpose();
        // step3:calculate R matrix
        Eigen::MatrixXf R_Matrix(2,2);
        R_Matrix(0,0) =  std::cos(phi0 - PI/2);
        R_Matrix(1,0) =  std::sin(phi0 - PI/2);
        R_Matrix(0,1) = -std::sin(phi0 - PI/2);
        R_Matrix(1,1) =  std::cos(phi0 - PI/2);
        // step4:calculate M matrix
        Eigen::MatrixXf M_Matrix(2,2);
        M_Matrix(0,0) =  -1/L0;
        M_Matrix(1,0) =  0;
        M_Matrix(0,1) =  0;
        M_Matrix(1,1) =  1;
        // step5:define F matrix, F = [Tp;F]
        Eigen::MatrixXf F_Matrix(2,1);
        float F_MatrixData[] = {Tp[i], F[i]};
        F_Matrix = Eigen::Map<Eigen::Matrix<float, 2, 1>>(F_MatrixData);  

        Eigen::MatrixXf out1_Matrix(2, 1);
        Eigen::MatrixXf out2_Matrix(2, 1);
        Eigen::MatrixXf out3_Matrix(2, 1);
        float out1_MatrixData[2] = {0, 0};
        float out2_MatrixData[2] = {0, 0};
        float out3_MatrixData[2] = {0, 0};      
        out1_Matrix = Eigen::Map<Eigen::Matrix<float, 2, 1>>(out1_MatrixData);
        out2_Matrix = Eigen::Map<Eigen::Matrix<float, 2, 1>>(out2_MatrixData);
        out3_Matrix = Eigen::Map<Eigen::Matrix<float, 2, 1>>(out3_MatrixData);  
        // step6:[TA;TE] = transpose(velocity_jacobin)*R*M*F
        out1_Matrix = M_Matrix * F_Matrix;
        out2_Matrix = R_Matrix * out1_Matrix;
        out3_Matrix = vJacobian_T_Matrix * out2_Matrix;
        
        torqueA[i] = out3_Matrix(0);
        torqueE[i] = out3_Matrix(1); 
    } 
}

// ==================== 状态空间方程 ====================

void WheelLeggedRobot::setStateSpaceMatrices(
    const Eigen::Matrix<float, 6, 6>& A,
    const Eigen::Matrix<float, 6, 2>& B
) {
    model_A_ = A;
    model_B_ = B;
}

void WheelLeggedRobot::calculateStateSpace(
    const std::array<std::array<float, 6>, 2>& x,
    const std::array<std::array<float, 2>, 2>& u,
    std::array<std::array<float, 6>, 2>& x_dot
) {
    for (int i = 0; i < 2; ++i) {
        // 转换为Eigen向量
        Eigen::Matrix<float, 6, 1> x_vec;
        Eigen::Matrix<float, 2, 1> u_vec;
        
        for (int j = 0; j < 6; ++j) {
            x_vec(j) = x[i][j];
        }
        for (int j = 0; j < 2; ++j) {
            u_vec(j) = u[i][j];
        }
        
        // 计算 x_dot = A*x + B*u
        Eigen::Matrix<float, 6, 1> x_dot_vec = model_A_ * x_vec + model_B_ * u_vec;
        
        // 转换回数组
        for (int j = 0; j < 6; ++j) {
            x_dot[i][j] = x_dot_vec(j);
        }
    }
}

// ==================== 状态更新 ====================

void WheelLeggedRobot::updateJointState(int joint_index, const Joint& joint) {
    joints_[joint_index] = joint;
}

void WheelLeggedRobot::updateBodyPose(
    const std::array<float, 3>& rpy,
    const std::array<float, 3>& w,
    const std::array<float, 3>& a
) {
    body_world_frame_.rpy = rpy;
    body_world_frame_.w = w;
    body_world_frame_.a = a;
}

void WheelLeggedRobot::updateAllJointState(const JointStateInterface& joints_state){
    if(joints_state.size() != 6){return;}

    for(size_t index = 0; index < 6; index++){
        Joint joint = {
            (config_.joints[index].invert_pos ? -1 : 1)*joints_state.getPosition(index) + config_.joints[index].pos_offset,
            (config_.joints[index].invert_vel ? -1 : 1)*joints_state.getVelocity(index),
            (config_.joints[index].invert_torque ? -1 : 1)*joints_state.getEffort(index)
        };
        updateJointState(index, joint);
    }
}


// ==================== 状态访问 ====================

std::array<WheelLeggedRobot::Joint, 4> WheelLeggedRobot::getHipJoints() const {
    std::array<WheelLeggedRobot::Joint, 4> hip_jonts = {
        joints_[LEFT_FRONT_HIP],  joints_[LEFT_BACK_HIP],
        joints_[RIGHT_FRONT_HIP], joints_[RIGHT_BACK_HIP]
    };
    return hip_jonts;
}

std::array<WheelLeggedRobot::Joint, 2> WheelLeggedRobot::getWheelJoints() const {
    std::array<WheelLeggedRobot::Joint, 2> wheel_joints = {
        joints_[LEFT_WHEEL],  
        joints_[RIGHT_WHEEL]
    };
    return wheel_joints;
}

std::array<WheelLeggedRobot::FiveLinkJointFrame, 2>
WheelLeggedRobot::getFiveLinkJointFrames() const {
    return fivelink_joint_frame_;
}

std::array<WheelLeggedRobot::VMCJointFrame, 2> 
WheelLeggedRobot::getVMCJointFrames() const {
    return vmc_joint_frame_;
}

std::array<WheelLeggedRobot::VMCForceFrame, 2> 
WheelLeggedRobot::getVMCForceFrames() const {
    return vmc_force_frame_;
}

std::array<WheelLeggedRobot::FiveLinkForceFrame, 2> 
WheelLeggedRobot::getFiveLinkForceFrames() const {
    return fivelink_force_frame_;
}

WheelLeggedRobot::BodyWorldFrame
WheelLeggedRobot::getBodyWorldFrame() const {
    return body_world_frame_;
}

// ==================== 调试打印 ====================

void WheelLeggedRobot::printModelParamInfo() const {
    std::cout << "\n========================================" << std::endl;
    std::cout << "五连杆轮腿机器人模型" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    
    std::cout << "\n[机体参数]" << std::endl;
    std::cout << "  质量: " << config_.body_link.mass << " kg" << std::endl;
    std::cout << "  pitch轴转动惯量: " << config_.body_link.pitch_inertia << " kg·m²" << std::endl;
    std::cout << "  yaw轴转动惯量: " << config_.body_link.yaw_inertia << " kg·m²" << std::endl;

    
    std::cout << "\n[连杆参数]" << std::endl;
    std::cout << "  大腿长度: " << config_.thigh_link[0].lengthOrRadius << " m" << std::endl;
    std::cout << "  小腿长度: " << config_.shank_link[0].lengthOrRadius << " m" << std::endl;
    std::cout << "  髋关节距离: " << config_.hip_joint_distance << " m" << std::endl;
    
    // for (int i = 0; i < 2; ++i) {
    //     std::cout << "\n[" << (i == 0 ? "左腿" : "右腿") << " 状态]" << std::endl;
    //     std::cout << "  L0: " << fivelink_joint_frame_[i].L0 << " m" << std::endl;
    //     std::cout << "  phi0: " << fivelink_joint_frame_[i].phi[0] * 180.0f / PI << " deg" << std::endl;
    //     std::cout << "  phi1: " << fivelink_joint_frame_[i].phi[1] * 180.0f / PI << " deg" << std::endl;
    //     std::cout << "  phi2: " << fivelink_joint_frame_[i].phi[2] * 180.0f / PI << " deg" << std::endl;
    //     std::cout << "  phi4: " << fivelink_joint_frame_[i].phi[4] * 180.0f / PI << " deg" << std::endl;
    // }
    
    std::cout << "========================================\n" << std::endl;
}

void WheelLeggedRobot::printModelStateInfo() const {
    std::cout << "\n========== EngineModel Debug Info ==========" << std::endl;
    
    // 打印身体姿态
    std::cout << "\n--- Body World Frame ---" << std::endl;
    std::cout << "Roll:  " << std::setw(10) << std::fixed << std::setprecision(4) << body_world_frame_.rpy[0] 
              << " rad, Pitch: " << std::setw(10) << body_world_frame_.rpy[1]
              << " rad, Yaw: " << std::setw(10) << body_world_frame_.rpy[2] << " rad" << std::endl;
    std::cout << "Angular Vel: [" 
              << std::setw(8) << body_world_frame_.w[0] << ", "
              << std::setw(8) << body_world_frame_.w[1] << ", "
              << std::setw(8) << body_world_frame_.w[2] << "] rad/s" << std::endl;
    std::cout << "Acceleration: [" 
              << std::setw(8) << body_world_frame_.a[0] << ", "
              << std::setw(8) << body_world_frame_.a[1] << ", "
              << std::setw(8) << body_world_frame_.a[2] << "] m/s²" << std::endl;
    
    // 打印关节状态（按左右侧）
    for (int side = 0; side < 2; side++) {
        std::cout << "\n--- " << (side == 0 ? "Left Side" : "Right Side") << " ---" << std::endl;
        
        // 5连杆关节帧
        std::cout << "FiveLink Frame:" << std::endl;
        std::cout << "  L0:  " << std::setw(10) << fivelink_joint_frame_[side].L0 << " m" << std::endl;
        std::cout << "  Phi: [";
        for (int i = 0; i < 5; i++) {
            std::cout << std::setw(8) << fivelink_joint_frame_[side].phi[i];
            if (i < 4) std::cout << ", ";
        }
        std::cout << "] rad" << std::endl;
        
        // VMC关节帧
        std::cout << "VMC Joint Frame:" << std::endl;
        std::cout << "  Theta: " << std::setw(10) << vmc_joint_frame_[side].theta 
                  << " rad, APhi: " << std::setw(10) << vmc_joint_frame_[side].alpha << " rad" << std::endl;
        
        // VMC力帧
        std::cout << "VMC Force Frame:" << std::endl;
        std::cout << "  Tp: " << std::setw(10) << vmc_force_frame_[side].Tp 
                  << " N·m, F: " << std::setw(10) << vmc_force_frame_[side].F << " N" << std::endl;
        

        if (side == 0) {  // 左: 0,3
            /// 髋关节
            std::cout << "  Joint 0: q=" << std::setw(8) << joints_[0].q 
                      << " rad, w=" << std::setw(8) << joints_[0].dq 
                      << " rad/s, t=" << std::setw(8) << joints_[0].tau << " N·m" << std::endl;
            std::cout << "  Joint 1: q=" << std::setw(8) << joints_[1].q 
                      << " rad, w=" << std::setw(8) << joints_[1].dq 
                      << " rad/s, t=" << std::setw(8) << joints_[1].tau << " N·m" << std::endl;    
            // 轮关节
            std::cout << "  Joint 2: q=" << std::setw(8) << joints_[2].q 
                      << " rad, w=" << std::setw(8) << joints_[2].dq 
                      << " rad/s, t=" << std::setw(8) << joints_[2].tau << " N·m" << std::endl;
        } else {  
            std::cout << "  Joint 3: q=" << std::setw(8) << joints_[3].q 
                      << " rad, w=" << std::setw(8) << joints_[3].dq 
                      << " rad/s, t=" << std::setw(8) << joints_[3].tau << " N·m" << std::endl;
            std::cout << "  Joint 4: q=" << std::setw(8) << joints_[4].q 
                      << " rad, w=" << std::setw(8) << joints_[4].dq 
                      << " rad/s, t=" << std::setw(8) << joints_[4].tau << " N·m" << std::endl;
            // 轮关节
            std::cout << "  Joint 5: q=" << std::setw(8) << joints_[5].q 
                      << " rad, w=" << std::setw(8) << joints_[5].dq 
                      << " rad/s, t=" << std::setw(8) << joints_[5].tau << " N·m" << std::endl;
        }
    }
    std::cout << "\n============================================\n" << std::endl;
}


// ==================== 初始化 ====================

void WheelLeggedRobot::initialize() {
    // 清零所有状态
    for (auto& joint : joints_) {
        joint = Joint();
    }
    
    for (auto& frame : fivelink_joint_frame_) {
        frame = FiveLinkJointFrame();
    }
    for (auto& frame : fivelink_force_frame_) {
        frame = FiveLinkForceFrame();
    }
    for (auto& frame : vmc_joint_frame_) {
        frame = VMCJointFrame();
    }
    for (auto& frame : vmc_force_frame_) {
        frame = VMCForceFrame();
    }
    
    body_world_frame_ = BodyWorldFrame();
    
    // 初始化状态空间矩阵
    model_A_.setZero();
    model_B_.setZero();
}

} // namespace robot