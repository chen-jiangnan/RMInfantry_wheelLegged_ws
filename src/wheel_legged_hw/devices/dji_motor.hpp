#pragma once
#include "dji_bsp.hpp"
#include "pid.hpp"

namespace dji_motor{

/* ================================================================
 *  M3508 电机
 *
 *  减速比：3591/187 ≈ 19.2
 *  Kt：    0.3 Nm/A
 *  最大电流：20A → 16384
 *
 *  控制模式：
 *   - 力矩控制：setControl(torque_nm)
 *   - 速度控制：setVelocityTarget(rad/s) + update(dt)  ← 串级内环
 * ================================================================ */
class M3508Motor : public DjiMotorBase {
public:
    M3508Motor(uint8_t           motor_id,
               MotorGroupSender& group,
               dm_tools::DMTools& dm,
               double  kReduction,
               bool              reversed = false)
        : DjiMotorBase(motor_id, 
                       (motor_id <= 4 ? motor_id-1 : motor_id-5), 
                       group, dm,
                       0x200 + motor_id,
                       kReduction,
                       reversed ? -1.0 : 1.0), kReduction_(kReduction)
    {}

    /* ---- PID 配置 ---- */
    void setVelocityPidParam(const control::PID::Param& p) {
        vel_pid_.setParam(p);
    }
    void resetPid() { vel_pid_.reset(); }

    /* ---- 速度控制接口 ---- */

    /** 设置目标速度（输出端 rad/s）*/
    void setVelocityTarget(double rad_s) { target_vel_ = rad_s; }

    /**
     * @brief 速度环 PID 更新，每控制周期调用一次
     *
     *   vel_target → vel_feedback(rad/s) → 力矩(Nm) → 电流原始值
     */
    void update(double dt) {
        if (!state_.online) return;
        double torque = vel_pid_.compute(
            target_vel_,      // 目标速度
            state_.velocity,  // 反馈速度
            dt);
        applyTorque(torque);
    }

    /* ---- 力矩控制（直接给输出端力矩 Nm）---- */
    void setTorque(double torque_nm){
        applyTorque(torque_nm);
    }

private:
    void applyTorque(double torque_nm) {
        double raw_d = torque_nm * sign_
                       / kReduction_ / kKt / kRawToAmp;
        setSlot(static_cast<int16_t>(
            std::clamp(raw_d,
                       -static_cast<double>(kMaxRaw),
                        static_cast<double>(kMaxRaw))));
    }

    void updateTorque() override {
        state_.torque = measure_.given_current
                        * kRawToAmp * kKt * kReduction_ * sign_;
    }

    double          target_vel_ = 0.0;
    control::PID    vel_pid_;

    double kReduction_ = 3591.0 / 187.0;                    // 减速箱*传动机构减速比 ！ ！ ！
    static constexpr double  kKt        = 0.015622389;      // 转子的扭矩系数 ！ ！ ！
    static constexpr double  kRawToAmp  = 20.0 / 16384.0;
    static constexpr int16_t kMaxRaw    = 16384;
};

/* ================================================================
 *  M2006 电机
 *
 *  减速比：36
 *  Kt：    0.18 Nm/A
 *  最大电流：10A → 10000
 * ================================================================ */
class M2006Motor : public DjiMotorBase {
public:
    M2006Motor(uint8_t           motor_id,
               uint8_t           slot,
               MotorGroupSender& group,
               dm_tools::DMTools& dm,
               bool              reversed = false)
        : DjiMotorBase(motor_id, slot, group, dm,
                       0x200 + motor_id,
                       kReduction,
                       reversed ? -1.0 : 1.0)
    {}

    /* ---- PID 配置 ---- */
    void setVelocityPidParam(const control::PID::Param& p) {
        vel_pid_.setParam(p);
    }
    void resetPid() { vel_pid_.reset(); }

    /* ---- 速度控制 ---- */
    void setVelocityTarget(double rad_s) { target_vel_ = rad_s; }

    void update(double dt) {
        if (!state_.online) return;
        double torque = vel_pid_.compute(target_vel_, state_.velocity, dt);
        applyTorque(torque);
    }

    /* ---- 力矩控制 ---- */
    void setControl(double torque_nm){
        applyTorque(torque_nm);
    }

private:
    void applyTorque(double torque_nm) {
        double raw_d = torque_nm * sign_
                       / kReduction / kKt / kRawToAmp;
        setSlot(static_cast<int16_t>(
            std::clamp(raw_d,
                       -static_cast<double>(kMaxRaw),
                        static_cast<double>(kMaxRaw))));
    }

    void updateTorque() override {
        state_.torque = measure_.given_current
                        * kRawToAmp * kKt * kReduction * sign_;
    }

    double          target_vel_ = 0.0;
    control::PID    vel_pid_;

    static constexpr double  kReduction = 36.0;
    static constexpr double  kKt        = 0.18;
    static constexpr double  kRawToAmp  = 10.0 / 10000.0;
    static constexpr int16_t kMaxRaw    = 10000;
};

} // namespace dji