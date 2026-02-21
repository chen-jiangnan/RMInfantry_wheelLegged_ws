#pragma once
#include "dm_tools.hpp"
#include <cstdint>
#include <cmath>
#include <array>

namespace dji_motor{

/* ================================================================
 *  转子端原始状态（所有 DJI 电机通用）
 * ================================================================ */
struct motor_measure_t {
    uint16_t ecd;             // 机械转角   0~8191
    int16_t  speed_rpm;       // 电机转速   rpm（转子）
    int16_t  given_current;   // 扭矩电流   原始值
    uint8_t  temperate;       // 温度       ℃
};

/* ================================================================
 *  输出端状态（SI 单位）
 * ================================================================ */
struct motor_state_t {
    double position = 0.0;   // 输出端累计角度  rad
    double velocity = 0.0;   // 输出端角速度    rad/s
    double torque   = 0.0;   // 输出端力矩      Nm
    bool   online   = false;
};

/* ================================================================
 *  组发送器
 *
 *  一帧控制 4 个电机槽位：
 *    0x200 → M3508/M2006  ID 1~4
 *    0x1FF → M3508/M2006  ID 5~8  /  GM6020 ID 1~4
 *    0x2FF → GM6020       ID 5~7
 * ================================================================ */
class MotorGroupSender {
public:
    MotorGroupSender(dm_tools::DMTools& dm,
                     uint8_t  channel,
                     uint32_t frame_id)
        : dm_(dm), channel_(channel), frame_id_(frame_id)
    {
        payload_.fill(0);
    }

    void setValue(uint8_t slot, int16_t value) {
        if (slot > 3) return;
        payload_[slot * 2]     = uint8_t(value >> 8);
        payload_[slot * 2 + 1] = uint8_t(value & 0xFF);
    }

    void send() {
        dm_.send(frame_id_, payload_.data(), 8, channel_, false, false, false);
    }

    void clear() { payload_.fill(0); }

private:
    dm_tools::DMTools& dm_;
    uint8_t                   channel_;
    uint32_t                  frame_id_;
    std::array<uint8_t, 8>    payload_;
};

/* ================================================================
 *  DJI 电机抽象基类
 *
 *  职责：
 *   - 管理 CAN 接收路由的注册 / 注销
 *   - 解析原始帧 → measure_（转子端原格式）
 *   - 将 measure_ 换算 → state_（输出端 SI）
 *   - 定义统一控制接口（子类实现）
 *
 *  子类只需要：
 *   - 提供 reduction / kt / recv_id_base 等参数
 *   - 实现 setControl() 控制接口
 * ================================================================ */
class DjiMotorBase {
public:
    DjiMotorBase(uint8_t           motor_id,
                 uint8_t           slot,
                 MotorGroupSender& group,
                 dm_tools::DMTools& dm,
                 uint32_t          recv_id,
                 double            reduction,
                 double            sign)
        : motor_id_(motor_id), slot_(slot),
          group_(group), dm_(dm),
          recv_id_(recv_id),
          reduction_(reduction), sign_(sign)
    {
        dm_.addRecvRoute(recv_id_,
            [this](uint32_t, const uint8_t* data, uint8_t) {
                canRecv(data);
            });
    }

    virtual ~DjiMotorBase() {
        dm_.removeRecvRoute(recv_id_);
    }

    // /* ---- 控制接口（子类实现） ---- */
    void clear() { group_.setValue(slot_, 0); }

    /* ---- 状态查询 ---- */
    const motor_measure_t& getMeasure() const { return measure_; }
    const motor_state_t&   getState()   const { return state_;   }
    uint8_t                getMotorId() const { return motor_id_; }

// protected:
    /* ---- 子类可调用：写组帧槽位 ---- */
    void setSlot(int16_t raw) { group_.setValue(slot_, raw); }

    uint8_t            motor_id_;
    uint8_t            slot_;
    MotorGroupSender&  group_;
    dm_tools::DMTools& dm_;
    uint32_t           recv_id_;
    double             reduction_;
    double             sign_;

    motor_measure_t    measure_{};
    motor_state_t      state_{};

private:
    /* ================================================================
     *  canRecv：所有 DJI 电机帧格式相同，基类统一解析
     *
     *  [0~1] ecd        0~8191
     *  [2~3] speed_rpm  rpm（转子）
     *  [4~5] current    原始值
     *  [6]   temperate  ℃
     * ================================================================ */
    void canRecv(const uint8_t* data) {
        // std::cout << "id:" << motor_id_ << " recv data:" << std::endl;
        /* 1. 填原始状态（保留原格式）*/
        measure_.ecd           = uint16_t(data[0]) << 8 | data[1];
        measure_.speed_rpm     = int16_t(uint16_t(data[2]) << 8 | data[3]);
        measure_.given_current = int16_t(uint16_t(data[4]) << 8 | data[5]);
        measure_.temperate     = data[6];

        /* 2. 转子端 → 输出端 SI */
        updateState();

        state_.online = true;
    }

    void updateState() {
        // 转子弧度
        double rotor_rad = measure_.ecd * (2.0 * M_PI / 8192.0);

        // 累计角度（跨圈）
        if (first_recv_) {
            last_rotor_rad_ = rotor_rad;
            first_recv_     = false;
        }
        double delta = rotor_rad - last_rotor_rad_;
        if (delta >  M_PI) delta -= 2.0 * M_PI;
        if (delta < -M_PI) delta += 2.0 * M_PI;
        accum_rotor_rad_ += delta;
        last_rotor_rad_   = rotor_rad;

        state_.position = accum_rotor_rad_ / reduction_ * sign_;
        state_.velocity = measure_.speed_rpm
                          / reduction_ / 60.0 * (2.0 * M_PI) * sign_;

        // 力矩由子类覆写（GM6020 无直接力矩换算）
        updateTorque();
    }

    // 子类可覆写力矩换算（默认为 0）
    virtual void updateTorque() { state_.torque = 0.0; }

    bool   first_recv_      = true;
    double last_rotor_rad_  = 0.0;
    double accum_rotor_rad_ = 0.0;
};

} // namespace dji