#pragma once
#include <cmath>
#include <algorithm>

namespace control {

/* ================================================================
 *  通用 PID 控制器
 *
 *  支持：
 *   - 积分限幅
 *   - 输出限幅
 *   - 微分滤波（一阶低通）
 *   - 角度误差处理（跨圈，±π）
 * ================================================================ */
class PID {
public:
    struct Param {
        double kp             = 0.0;
        double ki             = 0.0;
        double kd             = 0.0;
        double integral_limit = 0.0;   // 积分限幅，0 表示不限
        double output_limit   = 0.0;   // 输出限幅，0 表示不限
        double d_filter_coeff = 1.0;   // 微分滤波系数 0~1，1=不滤波
        bool   angle_mode     = false; // 角度模式（误差自动跨圈处理）
    };

    PID() = default;
    explicit PID(const Param& p) { setParam(p); }

    void setParam(const Param& p) { p_ = p; }

    void reset() {
        integral_   = 0.0;
        prev_error_ = 0.0;
        d_filtered_ = 0.0;
    }

    /**
     * @brief 计算 PID 输出
     * @param target   目标值
     * @param feedback 反馈值
     * @param dt       控制周期（秒）
     */
    double compute(double target, double feedback, double dt) {
        if (dt <= 0.0) return 0.0;

        double error = target - feedback;

        // 角度模式：将误差限制在 -π ~ π
        if (p_.angle_mode) {
            while (error >  M_PI) error -= 2.0 * M_PI;
            while (error < -M_PI) error += 2.0 * M_PI;
        }

        // 积分
        integral_ += error * dt;
        if (p_.integral_limit > 0.0)
            integral_ = std::clamp(integral_,
                                   -p_.integral_limit, p_.integral_limit);

        // 微分（带低通滤波）
        double d_raw  = (error - prev_error_) / dt;
        d_filtered_   = p_.d_filter_coeff * d_raw
                      + (1.0 - p_.d_filter_coeff) * d_filtered_;
        prev_error_   = error;

        double output = p_.kp * error
                      + p_.ki * integral_
                      + p_.kd * d_filtered_;

        // 输出限幅
        if (p_.output_limit > 0.0)
            output = std::clamp(output, -p_.output_limit, p_.output_limit);

        return output;
    }

    double getIntegral()  const { return integral_;   }
    double getPrevError() const { return prev_error_; }

private:
    Param  p_;
    double integral_   = 0.0;
    double prev_error_ = 0.0;
    double d_filtered_ = 0.0;
};

/* ================================================================
 *  串级 PID（外环位置 → 内环速度）
 *
 *  外环输出作为内环的目标值：
 *    outer.compute(pos_target, pos_feedback) → vel_target
 *    inner.compute(vel_target, vel_feedback) → control_output
 * ================================================================ */
class CascadePID {
public:
    struct Param {
        PID::Param outer;   // 外环（位置）
        PID::Param inner;   // 内环（速度）
    };

    explicit CascadePID(const Param& p = {}) {
        outer_.setParam(p.outer);
        inner_.setParam(p.inner);
    }

    void setParam(const Param& p) {
        outer_.setParam(p.outer);
        inner_.setParam(p.inner);
    }

    void reset() { outer_.reset(); inner_.reset(); }

    /**
     * @brief 串级计算
     * @param pos_target    外环目标（位置，rad）
     * @param pos_feedback  外环反馈（位置，rad）
     * @param vel_feedback  内环反馈（速度，rad/s）
     * @param dt            控制周期（秒）
     * @return              控制量（力矩 / 电压原始值，视电机类型）
     */
    double compute(double pos_target,
                   double pos_feedback,
                   double vel_feedback,
                   double dt)
    {
        double vel_target = outer_.compute(pos_target, pos_feedback, dt);
        return             inner_.compute(vel_target,  vel_feedback, dt);
    }

    PID& outer() { return outer_; }
    PID& inner() { return inner_; }

private:
    PID outer_;
    PID inner_;
};

} // namespace control