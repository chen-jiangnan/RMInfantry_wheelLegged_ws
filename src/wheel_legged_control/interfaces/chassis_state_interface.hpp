/**
 * @file chassis_state_interface.hpp
 * @brief ChassisState消息的C++封装接口（指针绑定版）
 */

#ifndef CHASSIS_STATE_INTERFACE_HPP
#define CHASSIS_STATE_INTERFACE_HPP

#include <array>
#include <cstddef>
#include "joint_fsm.hpp"
#include "Model/WheelLeggedRobot.hpp"
#include "Controller/LegController.hpp"
#include "Controller/LQRController.hpp"
#include "Controller/RotateController.hpp"
#include "Controller/StateEstimator.hpp"
#include "wheel_legged_msgs/msg/chassis_state.hpp"

namespace wheel_legged_interfaces {

/**
* @brief ChassisState接口类（指针绑定版）
* 
* 持有各控制器的 const 指针，toMsg() 直接从实例读取状态，
* 不需要手动调用 setter 同步数据。
*/
class ChassisStateInterface {
public:
    using msg_ChassisState = wheel_legged_msgs::msg::ChassisState;

    // ==================== 构造函数 ====================

    explicit ChassisStateInterface() = default;

    /**
    * @brief 绑定所有控制器指针
    * @note 调用方需保证指针生命周期长于本对象
    */
    void bind(
        const robot::WheelLeggedRobot*              robot,
        const controller::LegController*            leg_ctrl,
        const controller::LQRController6x2*         lqr_ctrl,
        const controller::RotateController*         rotate_ctrl,
        const controller::StateEstimator*           state_est
    ) {
        robot_          = robot;
        leg_ctrl_       = leg_ctrl;
        lqr_ctrl_       = lqr_ctrl;
        rotate_ctrl_    = rotate_ctrl;
        state_est_      = state_est;
    }

    // ==================== 转换方法 ====================

    /**
    * @brief 从绑定的控制器实例直接构建 msg
    */
    msg_ChassisState toMsg() const {
        msg_ChassisState msg;

        fillFsmMode(msg);
        fillRobotState(msg);
        fillLegCtrl(msg);
        fillLqrCtrl(msg);
        fillRotateCtrl(msg);
        fillStateEst(msg);

        return msg;
    }

    /**
    * @brief 从 msg 反序列化（用于订阅端）
    */
    void fromMsg(const msg_ChassisState& msg);

    void print() const;

private:
    JFSMode fsm_mode_;
    // ==================== 绑定指针 ====================
    const robot::WheelLeggedRobot*              robot_          = nullptr;
    const controller::LegController*            leg_ctrl_       = nullptr;
    const controller::LQRController6x2*         lqr_ctrl_       = nullptr;
    const controller::RotateController*         rotate_ctrl_    = nullptr;
    const controller::StateEstimator*           state_est_      = nullptr;

    // ==================== 私有填充方法 ====================

    void fillFsmMode(msg_ChassisState& msg) const {
        msg.joint_fsm_mode = static_cast<uint8_t>(fsm_mode_);
    }

    void fillRobotState(msg_ChassisState& msg) const {
        if (!robot_) return;

        const auto fivelink_jointframes = robot_->getFiveLinkJointFrames();
        const auto vmc_jointFrames      = robot_->getVMCJointFrames();
        const auto fivelink_forceframes = robot_->getFiveLinkForceFrames();
        const auto vmc_forceframes = robot_->getVMCForceFrames();
        const auto body      = robot_->getBodyWorldFrame();

        for (int leg = 0; leg < 2; ++leg) {
            msg.robot_state.fivelink_forceframe[leg].fx = fivelink_forceframes[leg].Fx;
            msg.robot_state.fivelink_forceframe[leg].fy = fivelink_forceframes[leg].Fy;
            msg.robot_state.legvmc_forceframe[leg].tp = vmc_forceframes[leg].Tp;
            msg.robot_state.legvmc_forceframe[leg].f = vmc_forceframes[leg].F;
            for (int i = 0; i < 5; ++i){
                msg.robot_state.fivelink_jointframe[leg].phi[i] = fivelink_jointframes[leg].phi[i];
            }
            msg.robot_state.fivelink_jointframe[leg].l0 = fivelink_jointframes[leg].L0;
            msg.robot_state.legvmc_jointframe[leg].theta = vmc_jointFrames[leg].theta;
            msg.robot_state.legvmc_jointframe[leg].alpha = vmc_jointFrames[leg].alpha;
        }
        for (int i = 0; i < 3; ++i) {
            msg.robot_state.body_worldframe.rpy[i] = body.rpy[i];
            msg.robot_state.body_worldframe.gyro[i] = body.w[i];
            msg.robot_state.body_worldframe.acc[i] = body.a[i];
        }
    }

    void fillLegCtrl(msg_ChassisState& msg) const {
        if (!leg_ctrl_) return;

        for (int i = 0; i < 2; ++i) {
            const auto& pid = leg_ctrl_->getSpringDampingPID(i);
            const auto& cfg = pid.getConfig();
            msg.leg_ctrl.spring_damping[i].kp       = cfg.kp;
            msg.leg_ctrl.spring_damping[i].ki       = cfg.ki;
            msg.leg_ctrl.spring_damping[i].kd       = cfg.kd;
            msg.leg_ctrl.spring_damping[i].max_iout = cfg.max_iout;
            msg.leg_ctrl.spring_damping[i].max_out  = cfg.max_out;
            msg.leg_ctrl.spring_damping[i].set      = pid.getSet(); 
            msg.leg_ctrl.spring_damping[i].fdb      = pid.getFdb();
            msg.leg_ctrl.spring_damping[i].p_out    = pid.getPOut();
            msg.leg_ctrl.spring_damping[i].i_out    = pid.getIOut();
            msg.leg_ctrl.spring_damping[i].d_out    = pid.getDOut();
            msg.leg_ctrl.spring_damping[i].out      = pid.getOutput();
        }

        {
            const auto& pid = leg_ctrl_->getRollCompensationPID();
            const auto& cfg = pid.getConfig();
            msg.leg_ctrl.compensation_roll.kp       = cfg.kp;
            msg.leg_ctrl.compensation_roll.ki       = cfg.ki;
            msg.leg_ctrl.compensation_roll.kd       = cfg.kd;
            msg.leg_ctrl.compensation_roll.max_iout = cfg.max_iout;
            msg.leg_ctrl.compensation_roll.max_out  = cfg.max_out;
            msg.leg_ctrl.compensation_roll.set      = pid.getSet(); 
            msg.leg_ctrl.compensation_roll.fdb      = pid.getFdb();
            msg.leg_ctrl.compensation_roll.p_out    = pid.getPOut();
            msg.leg_ctrl.compensation_roll.i_out    = pid.getIOut();
            msg.leg_ctrl.compensation_roll.d_out    = pid.getDOut();
            msg.leg_ctrl.compensation_roll.out      = pid.getOutput();
        }

        {
            const auto& pid = leg_ctrl_->getPhi0CompensationPID();
            const auto& cfg = pid.getConfig();
            msg.leg_ctrl.compensation_phi0.kp       = cfg.kp;
            msg.leg_ctrl.compensation_phi0.ki       = cfg.ki;
            msg.leg_ctrl.compensation_phi0.kd       = cfg.kd;
            msg.leg_ctrl.compensation_phi0.max_iout = cfg.max_iout;
            msg.leg_ctrl.compensation_phi0.max_out  = cfg.max_out;
            msg.leg_ctrl.compensation_phi0.set      = pid.getSet();
            msg.leg_ctrl.compensation_phi0.fdb      = pid.getFdb();
            msg.leg_ctrl.compensation_phi0.p_out    = pid.getPOut();
            msg.leg_ctrl.compensation_phi0.i_out    = pid.getIOut();
            msg.leg_ctrl.compensation_phi0.d_out    = pid.getDOut();
            msg.leg_ctrl.compensation_phi0.out      = pid.getOutput();
        }

        for (int i = 0; i < 2; ++i) {
            msg.leg_ctrl.gravity_compensation[i] = leg_ctrl_->getGravityCompensation()[i];
            msg.leg_ctrl.feedforward_fs[i] = leg_ctrl_->getFeedforwardForce()[i];
            msg.leg_ctrl.compensation_tp[i] = leg_ctrl_->getOutPutTp()[i];
            msg.leg_ctrl.f[i] = leg_ctrl_->getOutPutF()[i];
        }
    }

    void fillLqrCtrl(msg_ChassisState& msg) const {
        if (!lqr_ctrl_) return;
        const auto& cfg = lqr_ctrl_->getConfig();
        for(int i = 0; i < 12; i++){
            for(int j = 0; j < 3; j++){
                msg.lqr_ctrl.fit_coeffs[i*3 + j] = cfg.fit_coeffs[i][j];
            }
        }
        const auto& k_l   = lqr_ctrl_->getGainMatrix(0);
        const auto& x_l   = lqr_ctrl_->getXMatrix(0);
        const auto& xd_l  = lqr_ctrl_->getXdMatrix(0);
        const auto& err_l = lqr_ctrl_->getErrMatrix(0);
        const auto& u_l   = lqr_ctrl_->getUMatrix(0);

        const auto& k_r   = lqr_ctrl_->getGainMatrix(1);
        const auto& x_r   = lqr_ctrl_->getXMatrix(1);
        const auto& xd_r  = lqr_ctrl_->getXdMatrix(1);
        const auto& err_r = lqr_ctrl_->getErrMatrix(1);
        const auto& u_r   = lqr_ctrl_->getUMatrix(1);

        for (int i = 0; i < 6; i++){
            msg.lqr_ctrl.x_l[i] =  x_l(i);
            msg.lqr_ctrl.xd_l[i] =  xd_l(i);
            msg.lqr_ctrl.err_l[i] =  err_l(i);
            msg.lqr_ctrl.x_r[i] =  x_r(i);
            msg.lqr_ctrl.xd_r[i] =  xd_r(i);
            msg.lqr_ctrl.err_r[i] =  err_r(i);
            
        }

        for(int i = 0; i < 2; i++){
            msg.lqr_ctrl.u_l[i] =  u_l(i);
            msg.lqr_ctrl.u_r[i] =  u_r(i);
        }


        for(int i = 0; i < 12; i++){
            msg.lqr_ctrl.k_l[i] =  k_l(i);
            msg.lqr_ctrl.k_r[i] =  k_r(i);
        }

    }

    void fillRotateCtrl(msg_ChassisState& msg) const {
        if (!rotate_ctrl_) return;
        const auto& pid = rotate_ctrl_->getPID();
        const auto& cfg = pid.getConfig();
        msg.rotate_ctrl.yaw_rotate.kp        = cfg.kp;
        msg.rotate_ctrl.yaw_rotate.ki        = cfg.ki;
        msg.rotate_ctrl.yaw_rotate.kd        = cfg.kd;
        msg.rotate_ctrl.yaw_rotate.max_iout  = cfg.max_iout;
        msg.rotate_ctrl.yaw_rotate.max_out   = cfg.max_out;
        msg.rotate_ctrl.yaw_rotate.set      = pid.getSet();
        msg.rotate_ctrl.yaw_rotate.fdb      = pid.getFdb();
        msg.rotate_ctrl.yaw_rotate.p_out     = pid.getPOut();
        msg.rotate_ctrl.yaw_rotate.i_out     = pid.getIOut();
        msg.rotate_ctrl.yaw_rotate.d_out     = pid.getDOut();
        msg.rotate_ctrl.yaw_rotate.out       = pid.getOutput();
    }

    void fillStateEst(msg_ChassisState& msg) const {
        if (!state_est_) return;

        const auto& force = state_est_->getForceEstimate();
        const auto& vel   = state_est_->getVelocityEstimate();
        const auto& kf    = state_est_->getKalmanFilter();
        for (int i = 0; i < 2; ++i) {
            msg.state_est.fn_est.ddot_zw[i] = force.ddot_zw[i];
            msg.state_est.fn_est.fn[i]       = force.Fn[i];
        }
        msg.state_est.fn_est.ddot_zb = force.ddot_zb;
        msg.state_est.fn_est.ground_signal = force.ground_signal_;

        
        for (int i = 0; i < 2; ++i) {
            msg.state_est.xv_est.dot_xw[i]   = vel.dot_xw[i];
            msg.state_est.xv_est.slip_signal[i] = vel.slip_signal_[i];
        }
        msg.state_est.xv_est.aver_vel = vel.aver_vel;
        msg.state_est.xv_est.x_filter = vel.x_filter;
        msg.state_est.xv_est.v_filter = vel.v_filter;

        // ========== 状态量 ==========
        msg.state_est.xv_est.kalman_filter.position = kf.x(0);     // x[0] 位置 (k|k)
        msg.state_est.xv_est.kalman_filter.velocity = kf.x(1);     // x[1] 速度 (k|k)

        // ========== 协方差对角元素 ==========
        msg.state_est.xv_est.kalman_filter.p_pos_pos = kf.P(0,0);    // P[0,0] 位置方差
        msg.state_est.xv_est.kalman_filter.p_vel_vel = kf.P(1,1);    // P[1,1] 速度方差
        msg.state_est.xv_est.kalman_filter.p_pos_vel = kf.P(0,1);    // P[0,1] 交叉项

        // ========== 量测相关 ==========
        msg.state_est.xv_est.kalman_filter.z_raw_0   = kf.measured_vector(0);   // measured_vector[0] 原始量测 (速度项)
        msg.state_est.xv_est.kalman_filter.z_raw_1   = kf.measured_vector(1);   // measured_vector[1] 原始量测 （加速度项）

        // auto z_raw = kf.getZVector() - kf.getHMatrix()*kf.x;
        // msg.state_est.xv_est.kalman_filter.innovation_0  = z_raw(0);// 新息 z - H*x（量测残差）
        // msg.state_est.xv_est.kalman_filter.innovation_1  = z_raw(1)
        
        // ========== 卡尔曼增益对角 ==========
        // msg.state_est.xv_est.kalman_filter.k_pos     = kf.getKMatrix()(0);     // K[0,*] 对位置的增益
        // msg.state_est.xv_est.kalman_filter.k_vel     = kf.getKMatrix()(0);     // K[1,*] 对速度的增益

        // ========== 标志位 ==========
        // msg.state_est.xv_est.kalman_filter.use_auto_adjustment
        // msg.state_est.xv_est.kalman_filter.uint8 valid_measurement_num   // valid_z_num_
    }
};

} // namespace wheel_legged_interfaces

#endif // CHASSIS_STATE_INTERFACE_HPP