/**
 * @file WheelLeggedRobot.hpp
 * @brief 五连杆轮腿机器人完整实现 
 * 
 * 完整实现所有五连杆运动学、动力学计算
 * 基于VMC (Virtual Model Control) 理论
 */

#ifndef WHEEL_LEGGED_ROBOT_HPP
#define WHEEL_LEGGED_ROBOT_HPP

#include "joint_state_interface.hpp"
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <vector>
 
namespace robot {

constexpr float G = 9.8f;
constexpr float PI = 3.14159265358979323846f;
constexpr int HISTORY_NUMS = 10;

/**
* @brief 五连杆轮腿机器人类
* 
* 五连杆构型 (平行四边形):
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
* - A: 前髋关节 (phi1)
* - E: 后髋关节 (phi4)  
* - C: 踝关节 (轮轴心)
* - l1 = l4 = L_thigh (大腿长)
* - l2 = l3 = L_shank (小腿长)
*/
class WheelLeggedRobot {
public:
    /**
     * @brief 关节索引
     * 
     * 统一管理所有关节
     */
    enum JointIndex : uint8_t {
        // 左腿
        LEFT_FRONT_HIP  = 0,  // 左前髋
        LEFT_BACK_HIP   = 1,  // 左后髋
        LEFT_WHEEL      = 2,  // 左轮
        // 右腿
        RIGHT_FRONT_HIP = 3,  // 右前髋
        RIGHT_BACK_HIP  = 4,  // 右后髋
        RIGHT_WHEEL     = 5,  // 右轮
        // 总数
        TOTAL_JOINTS    = 6
    };

    const std::vector<std::string> join_names = {
        "LEFT_FRONT_HIP",
        "LEFT_BACK_HIP",
        "LEFT_WHEEL",
        "RIGHT_FRONT_HIP",
        "RIGHT_BACK_HIP",
        "RIGHT_WHEEL",
    };

    /**
    * @brief 连杆参数
    */
    struct Link {
        float mass = 0.0f;               //质量描述              单位:kg
        float lengthOrRadius = 0.0f;     //长度或者半径描述       单位:m
        float inertia = 0.0f;            //转动惯量描述(绕质心)   单位:kg*m^2 
    };
    
    /**
    * @brief 机体连杆
    */
    struct BodyLink {
        float mass = 0.0f;              //质量描述                        单位:kg
        float l = 0.0f;                 //虚拟连杆的转轴到机体质心的距离      单位:m
        float pitch_inertia = 0.0f;     //转动惯量描述                     单位:kg*m^2
        float yaw_inertia = 0.0f;       //转动惯量描述                     单位:kg*m^2
        float theta_b0 = 0.0f;          //机体质心偏角
    };
    
    /**
    * @brief 虚拟连杆
    */
    struct VirtualLink {
        float mass = 0.0f;              //质量描述                      单位:kg
        float L = 0.0f;                 //车轮轴心到虚拟连杆质心的距离     单位:m
        float LM = 0.0f;                //虚拟杆质心到虚拟杆转轴的距离     单位:m
        float inertia = 0.0f;           //转动惯量描述                  单位:kg*m^2
    };

    /**
    * @brief 关节状态
    */
    struct Joint {
        float q   = 0.0f;  //关节角位置, 电机反馈值    单位:rad
        float dq  = 0.0f;  //关节角速度, 电机反馈值    单位:rad/s
        float tau = 0.0f;  //关节角力矩, 电机反馈值    单位:n*m
    };

    /**
    * @brief 关节配置
    */
    struct JointConfig {
        JointIndex index;
        bool invert_pos = false;
        bool invert_vel = false;
        bool invert_torque = false;
        float pos_offset = 0.0f;
    };
    
    /**
    * @brief 机器人配置
    */
    struct Config {
        // 连杆参数
        BodyLink body_link;                         //机体
        std::array<Link, 2> thigh_link;             //大腿
        std::array<Link, 2> shank_link;             //小腿
        std::array<Link, 2> wheel_link;             //轮
        std::array<VirtualLink, 2> virtual_link;    //摆杆
    
        std::vector<std::array<float, 4>> leg_dynamics_table;//
        
        // 几何参数
        float hip_joint_offset = 0.0f;      // 大腿连杆偏角（rad）
        float hip_joint_distance = 0.0f;    // 前后髋关节距离 (m)
        float half_wheel_track = 0.0f;      // 半轮距 (m)

        // 关节参数
        std::array<JointConfig, 6> joints; 
        void updateJointOffsets(){
            joints = {{
                {LEFT_FRONT_HIP,  false, false, false,    - hip_joint_offset},
                {LEFT_BACK_HIP,   false, false, false, PI + hip_joint_offset},
                {LEFT_WHEEL,      false, false, false, 0.0f},
                {RIGHT_FRONT_HIP, true,  true,  true,     - hip_joint_offset},
                {RIGHT_BACK_HIP,  true,  true,  true,  PI + hip_joint_offset},
                {RIGHT_WHEEL,     true,  true,  true,  0.0f}
            }};
        }
        Config() = default;
    };
    
    /**
    * @brief 五连杆关节坐标系
    */
    struct FiveLinkJointFrame {
        float L0 = 0.0f;                          //虚拟杆长                                                                     单位:m
        std::array<float, 5> phi{0.0f};           //5连杆各杆在该坐标系下与x轴的角度，其中index:0为虚拟杆长,index:1~4从A点按顺时针编号    单位:rad
        std::array<float, HISTORY_NUMS> L0_history{0.0f};   // 腿长历史
    };
    
    /**
    * @brief VMC关节坐标系
    */
    struct VMCJointFrame {
        float theta = 0.0f;  //虚拟杆在世界坐标系下与竖直方向的夹角      (rad)
        float alpha = 0.0f;  //虚拟杆在机体坐标系下与竖直方向的夹角      (rad)
        std::array<float, HISTORY_NUMS> theta_history{0.0f};
        std::array<float, HISTORY_NUMS> alpha_history{0.0f};
    };

    /**
    * @brief 五连杆力坐标系
    */
    struct FiveLinkForceFrame {
        float Fx = 0.0f;  //5连杆坐标系下，踝关节处沿x轴方向的力 (N)
        float Fy = 0.0f;  //5连杆坐标系下，踝关节处沿y轴方向的力 (N)
    };

    /**
    * @brief VMC力坐标系
    */
    struct VMCForceFrame {
        float Tp = 0.0f;  //虚拟连杆坐标系下绕机体转轴旋转的力矩  (N·m)
        float F = 0.0f;   //虚拟连杆坐标系下沿杆长方向维持杆长的力  (N)
    };
    
    /**
    * @brief 机体世界坐标系
    */
    struct BodyWorldFrame {
        std::array<float, 3> rpy{0.0f};  // roll, pitch, yaw (rad)
        std::array<float, 3> w{0.0f};    // 角速度 (rad/s)
        std::array<float, 3> a{0.0f};    // 角加速度 (rad/s²)
    };
    
    
    // ==================== 构造函数 ====================
    
    WheelLeggedRobot();
    explicit WheelLeggedRobot(const Config& config);
    ~WheelLeggedRobot() = default;
    
    // ==================== 配置管理 ====================
    
    void setConfig(const Config& config);
    const Config& getConfig() const;
    
    // ==================== 正逆运动学 ====================
    
    /**
    * @brief   基于VMC的5连杆正运动学，输入5连杆笛卡尔坐标系下的[phi1,phi4] -> 输出5连杆极坐标系下的[phi0, L0]
    * @note    1.正向运动学默认更新model里的FiveLinkJointFrame
    *          2.const 修饰的形参为正运动学输入, 非const 修饰的形参为正运动学输出
    * @param[in]    phi1: 正运动学输入，长度为二的数组(index:0->left, index:1->right)，front髋关节电机力矩
    * @param[in]    phi4: 正运动学输入，长度为二的数组(index:0->left, index:1->right)，behind髋关节电机力矩
    * @param[out]   phi0: 正运动学输出，长度为二的数组(index:0->left, index:1->right)，FiveLinkJointFrame极坐标系下对应为phi0
    * @param[out]   L0:   正运动学输出，长度为二的数组(index:0->left, index:1->right)，FiveLinkJointFrame极坐标系下对应为L0
    * @param[in]    update_frame: 布尔值，为True则更新FiveLinkJointFrame
    */
    void forwardKinematics(
        const std::array<float, 2>& phi1,
        const std::array<float, 2>& phi4,
        std::array<float, 2>& phi0,
        std::array<float, 2>& L0,
        bool update_frame = true
    );
    
    /**
    * @brief   基于VMC的5连杆逆运动学，输入5连杆极坐标系下的[phi0, L0] -> 输出5连杆笛卡尔坐标系下的[phiA,phiE]
    * @note    1.逆向运动学默认不更新model里的fiveLink_jointFrame
    *          2.const 修饰的形参为逆运动学输入, 非const 修饰的形参为逆运动学输出
    * @param[in]    phi0:逆运动学输入，fiveLink_jointFrame极坐标系下对应phi0
    * @param[in]    L0:逆运动学输入，fiveLink_jointFrame极坐标系下对应为L0
    * @param[out]   phi1:逆运动学输出, fiveLink_jointFrame笛卡尔坐标系下对应为phiA
    * @param[out]   phi4:逆运动学输出, fiveLInk_jointFrame笛卡尔坐标系下对应为phiE
    */
    void inverseKinematics(
        const std::array<float, 2>& phi0,
        const std::array<float, 2>& L0,
        std::array<float, 2>& phi1,
        std::array<float, 2>& phi4
    );
    
    // ==================== 正逆动力学 ====================
    
    /**
    * @brief   基于VMC的5连杆正动力学，输入5连杆笛卡尔坐标系下的[torqueA,torqueE] -> 输出5连杆极坐标系下的[Tp, F]
    * @note    1.正向动力学默认更新model里的fiveLink_forceFrame
    *          2.const 修饰的形参为正动力学输入, 非const 修饰的形参为正动力学输出
    * @param[in]   torqueA: 正动力学输入，长度为二的数组(index:0->left, index:1->right), 髋关节电机力矩, 对应5连杆坐标系下的A
    * @param[in]   torqueE: 正动力学输入，长度为二的数组(index:0->left, index:1->right)，髋关节电机力矩, 对应5连杆坐标系下的E
    * @param[out]  Tp:  正动力学输出，长度为二的数组(index:0->left, index:1->right)，绕机体转轴的虚拟力矩
    * @param[out]  F:   正动力学输出，长度为二的数组(index:0->left, index:1->right), 沿杆长方向的虚拟力
    * @param[in]   ifUpdate: 布尔值，ifUpdate为True则更新fiveLink_forceFrame
    */
    void forwardDynamics(
        const std::array<float, 2>& torqueA,
        const std::array<float, 2>& torqueE,
        std::array<float, 2>& Tp,
        std::array<float, 2>& F,
        bool update_frame = true
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
    void inverseDynamics(
        const std::array<float, 2>& Tp,
        const std::array<float, 2>& F,
        std::array<float, 2>& torqueA,
        std::array<float, 2>& torqueE
    );
    
    // ==================== 状态空间方程 ====================
    
    /**
    * @brief 设置状态空间矩阵
    */
    void setStateSpaceMatrices(
        const Eigen::Matrix<float, 6, 6>& A,
        const Eigen::Matrix<float, 6, 2>& B
    );
    
    /**
    * @brief   轮式5连杆构型机器人状态空间方程运算
    *          X_dot = A*X + B*U
    * @note    1.x u x_dot 均为1维数组
    * @param[in]    x:   系统状态矩阵 维度：6x1 [theta theta_dot x x_dot phi phi_dot]
    * @param[in]    u:   系统输入矩阵 维度: 2x1 [Tp T]
    * @param[out]   x_dot:系统状态微分矩阵 维度:6x1 
    */
    void calculateStateSpace(
        const std::array<std::array<float, 6>, 2>& x,
        const std::array<std::array<float, 2>, 2>& u,
        std::array<std::array<float, 6>, 2>& x_dot
    );
    
    
    // ==================== 状态访问 ====================
    /**
    * @brief 更新关节状态
    */
    using JointStateInterface = wheel_legged_interfaces::JointStateInterface;
    void updateJointState(int joint_index, const Joint& joint);
    void updateAllJointState(const JointStateInterface& joints);
    void updateBodyPose(const std::array<float, 3>& rpy,
                        const std::array<float, 3>& w = {0.0f, 0.0f, 0.0f},
                        const std::array<float, 3>& a = {0.0f, 0.0f, 0.0f});
    
    std::array<Joint, 4> getHipJoints() const;
    std::array<Joint, 2> getWheelJoints() const;
    std::array<FiveLinkJointFrame, 2> getFiveLinkJointFrames() const;
    std::array<VMCJointFrame, 2> getVMCJointFrames() const;
    std::array<VMCForceFrame, 2> getVMCForceFrames() const;
    std::array<FiveLinkForceFrame, 2> getFiveLinkForceFrames() const;
    BodyWorldFrame getBodyWorldFrame() const;
    
    // ==================== 调试打印 ====================
    
    void printModelParamInfo() const;
    void printModelStateInfo() const;

private:
    // ==================== 私有方法 ====================

    /**
    * @brief 初始化
    */    
    void initialize();

    
    // ==================== 成员变量 ====================
    
    Config config_;
    
    // 关节状态
    std::array<Joint, 6> joints_;
    
    // 坐标系
    std::array<FiveLinkJointFrame, 2> fivelink_joint_frame_;
    std::array<FiveLinkForceFrame, 2> fivelink_force_frame_;
    std::array<VMCJointFrame, 2> vmc_joint_frame_;
    std::array<VMCForceFrame, 2> vmc_force_frame_;
    BodyWorldFrame body_world_frame_;
    
    // 状态空间模型
    Eigen::Matrix<float, 6, 6> model_A_;
    Eigen::Matrix<float, 6, 2> model_B_;
};

} // namespace robot

#endif // WHEEL_LEGGED_ROBOT_HPP