#pragma once
#include <unordered_map>
#include <cstring>
#include "dm_tools.hpp"

namespace dm_motor {

struct MotorState {
    float   position  = 0.0f;
    float   velocity  = 0.0f;
    float   torque    = 0.0f;
    float   temperate = 0.0f;
    uint8_t errcode   = 0;
    bool    online    = false;
};

enum DM_Motor_Type : uint8_t {
    DM3507=0, DM4310, DM4310_48V, DM4340, DM4340_48V,
    DM6006, DM6248, DM8006, DM8009, DM10010L, DM10010,
    DMH3510, DMH6215, DMS3519, DMG6220, Num_Of_Motor
};
struct LimitParam { float Q_MAX, DQ_MAX, TAU_MAX; };
static constexpr LimitParam kLimitTable[Num_Of_Motor] = {
    {12.566f, 50, 5},   {12.5f, 30, 10},  {12.5f, 50, 10},
    {12.5f, 10, 28},    {12.5f, 20, 28},  {12.5f, 45, 12},
    {12.566f, 20, 120}, {12.5f, 45, 20},  {12.5f, 45, 54},
    {12.5f, 25, 200},   {12.5f, 20, 200}, {12.5f, 280, 1},
    {12.5f, 45, 10},    {12.5f, 2000, 2}, {12.5f, 45, 10}
};

enum ModeOffset : uint16_t {
    MIT_MODE=0x000, POS_VEL_MODE=0x100, VEL_MODE=0x200, POS_FORCE_MODE=0x300
};

namespace utils {
    inline uint16_t f2u(float x, float xmin, float xmax, uint8_t bits) {
        return uint16_t((x-xmin)/(xmax-xmin)*((1u<<bits)-1u));
    }
    inline float u2f(uint16_t x, float xmin, float xmax, uint8_t bits) {
        return float(x)/float((1u<<bits)-1u)*(xmax-xmin)+xmin;
    }
}

class DamiaoMotor {
public:
    DamiaoMotor(uint16_t motor_id, uint16_t master_id,
                dm_tools::DMTools& dm,
                DM_Motor_Type type,
                ModeOffset init_mode,
                uint8_t channel = 0)
        : dm_(dm), channel_(channel), motor_id_(motor_id), master_id_(master_id),
        mode_(init_mode) , limit_(kLimitTable[type])
    {
        can_id_ = motor_id_ + mode_;

        // 构造时自动注册接收路由
        dm_.addRecvRoute(master_id_, [this](uint32_t, const uint8_t* data, uint8_t) {
            canRecv(data);
        });
    }

    ~DamiaoMotor() {
        dm_.removeRecvRoute(master_id_);
    }

    void enable()          { uint8_t d[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC}; canSend(can_id_,d,8); }
    void disable()         { uint8_t d[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD}; canSend(can_id_,d,8); }
    void setZeroPosition() { uint8_t d[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE}; canSend(can_id_,d,8); }

    void controlMIT(float kp, float kd, float pos, float vel, float tau) {
        using namespace utils;
        
        pos = std::clamp(pos, -limit_.Q_MAX,  limit_.Q_MAX);
        vel = std::clamp(vel, -limit_.DQ_MAX, limit_.DQ_MAX);
        tau = std::clamp(tau, -limit_.TAU_MAX, limit_.TAU_MAX);
        kp  = std::clamp(kp,  0.f, 500.f);
        kd  = std::clamp(kd,  0.f, 5.f);
    
        uint16_t kp_u=f2u(kp,0,500,12), kd_u=f2u(kd,0,5,12);
        uint16_t p_u=f2u(pos,-limit_.Q_MAX,limit_.Q_MAX,16);
        uint16_t v_u=f2u(vel,-limit_.DQ_MAX,limit_.DQ_MAX,12);
        uint16_t t_u=f2u(tau,-limit_.TAU_MAX,limit_.TAU_MAX,12);

        uint8_t d[8]={uint8_t(p_u>>8),uint8_t(p_u),uint8_t(v_u>>4),
        uint8_t((v_u&0xf)<<4|(kp_u>>8)),uint8_t(kp_u),uint8_t(kd_u>>4),
        uint8_t((kd_u&0xf)<<4|(t_u>>8)),uint8_t(t_u)};
        canSend(can_id_, d, 8);
    }

    void controlVelocity(float vel) {
        uint8_t d[4]; std::memcpy(d,&vel,4); canSend(can_id_,d,4);
    }

    void controlPositionVelocity(float pos, float vel) {
        uint8_t d[8]; std::memcpy(d,&pos,4); std::memcpy(d+4,&vel,4); canSend(can_id_,d,8);
    }

    const MotorState& getState()    const { return state_;     }
    uint32_t          getMotorId()  const { return motor_id_;  }
    uint32_t          getCanId()    const { return can_id_;    }
    uint32_t          getMasterId() const { return master_id_; }

private:
    void canSend(uint32_t frame_id, uint8_t* data, uint8_t len) {
        dm_.send(frame_id, data, len, channel_, false, false, false);
    }

    void canRecv(const uint8_t* data) {
        // std::cout << "id:" << motor_id_ << " recv data:" << std::endl;
        using namespace utils;
        uint16_t q_u   = (uint16_t(data[1])<<8)|data[2];
        uint16_t dq_u  = (uint16_t(data[3])<<4)|(data[4]>>4);
        uint16_t tau_u = (uint16_t(data[4]&0xf)<<8)|data[5];
        state_.position  = u2f(q_u,  -limit_.Q_MAX,  limit_.Q_MAX,  16);
        state_.velocity  = u2f(dq_u, -limit_.DQ_MAX, limit_.DQ_MAX, 12);
        state_.torque    = u2f(tau_u,-limit_.TAU_MAX, limit_.TAU_MAX,12);
        state_.temperate = static_cast<float>(data[6]);
        state_.online    = true;
    }

    dm_tools::DMTools&   dm_;
    uint8_t    channel_;
    uint32_t   motor_id_, can_id_, master_id_;
    MotorState state_;
    ModeOffset mode_;
    LimitParam limit_;
};

} // namespace damiao