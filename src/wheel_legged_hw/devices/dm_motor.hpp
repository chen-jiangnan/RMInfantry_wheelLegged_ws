#pragma once
#include <unordered_map>
#include <cstring>

extern "C" {
    #include "pub_user.h"
}


namespace damiao {

/**
 * @brief 电机状态数据（所有电机通用）
 */
 struct MotorState {
    float position  = 0.0f;   // rad
    float velocity  = 0.0f;   // rad/s
    float torque    = 0.0f;   // Nm
    float temperate = 0.0f;   // ℃
    uint8_t errcode = 0;      // 错误码
    bool  online    = false;  // 
};

enum DM_Motor_Type : uint8_t {
    DM3507=0, DM4310, DM4310_48V, DM4340, DM4340_48V,
    DM6006, DM6248, DM8006, DM8009, DM10010L, DM10010,
    DMH3510, DMH6215, DMS3519, DMG6220, Num_Of_Motor
};
struct LimitParam { float Q_MAX, DQ_MAX, TAU_MAX; };
static constexpr LimitParam kLimitTable[Num_Of_Motor] = {
    {12.566, 50, 5},  // DM3507          check 
    {12.5, 30, 10},   // DM4310          check
    {12.5, 50, 10},   // DM4310_48V
    {12.5, 10, 28},   // DM4340          check
    {12.5, 20, 28},   // DM4340_48V      check
    {12.5, 45, 12},   // DM6006          check
    {12.566, 20, 120},   // DM6248       check
    {12.5, 45, 20},   // DM8006          check
    {12.5, 45, 54},   // DM8009          check
    {12.5, 25, 200},  // DM10010L        check
    {12.5, 20, 200},  // DM10010         check
    {12.5, 280, 1},   // DMH3510         check
    {12.5, 45, 10},   // DMH6215
    {12.5, 2000, 2},  // DMS3519         check
    {12.5, 45, 10}    // DMG6220         check
};

enum ModeOffset : uint16_t {
    MIT_MODE=0x000, POS_VEL_MODE=0x100, VEL_MODE=0x200, POS_FORCE_MODE=0x300
};

namespace utils {
    inline uint16_t f2u(float x,float xmin,float xmax,uint8_t bits){
        return uint16_t((x-xmin)/(xmax-xmin)*((1u<<bits)-1u));
    }
    inline float u2f(uint16_t x,float xmin,float xmax,uint8_t bits){
        return float(x)/float((1u<<bits)-1u)*(xmax-xmin)+xmin;
    }
}

class DamiaoMotor{
public:
    DamiaoMotor(uint16_t motor_id, uint16_t master_id, 
        device_handle* dm_device, DM_Motor_Type type, ModeOffset init_mode)
        :   
            dm_device_(dm_device),
            motor_id_(motor_id), 
            mode_(init_mode), 
            limit_(kLimitTable[type]) {
            can_id_    = motor_id_ + mode_;
            master_id_ = master_id;
        }

    /* -------- 生命周期 -------- */
    void enable() { 
        uint8_t payload_tx_[8] = {0};
        payload_tx_[0] = 0xFF;
        payload_tx_[1] = 0xFF;
        payload_tx_[2] = 0xFF;
        payload_tx_[3] = 0xFF;
        payload_tx_[4] = 0xFF;
        payload_tx_[5] = 0xFF;
        payload_tx_[6] = 0xFF;
        payload_tx_[7] = 0xFC;

        canSend(can_id_, payload_tx_);
    }
    void disable() { 
        uint8_t payload_tx_[8] = {0};

        payload_tx_[0] = 0xFF;
        payload_tx_[1] = 0xFF;
        payload_tx_[2] = 0xFF;
        payload_tx_[3] = 0xFF;
        payload_tx_[4] = 0xFF;
        payload_tx_[5] = 0xFF;
        payload_tx_[6] = 0xFF;
        payload_tx_[7] = 0xFD;

        canSend(can_id_, payload_tx_);
    }
    void setZeroPosition() { 
        uint8_t payload_tx_[8] = {0};

        payload_tx_[0] = 0xFF;
        payload_tx_[1] = 0xFF;
        payload_tx_[2] = 0xFF;
        payload_tx_[3] = 0xFF;
        payload_tx_[4] = 0xFF;
        payload_tx_[5] = 0xFF;
        payload_tx_[6] = 0xFF;
        payload_tx_[7] = 0xFE; 

        canSend(can_id_, payload_tx_);
    }

    /* -------- 控制 -------- */
    void controlMIT(float kp,float kd,float pos,float vel,float tau)  {
        using namespace utils;
        uint16_t kp_u =f2u(kp, 0,500,12), kd_u=f2u(kd,0,5,12);
        uint16_t p_u  =f2u(pos,-limit_.Q_MAX,  limit_.Q_MAX,  16);
        uint16_t v_u  =f2u(vel,-limit_.DQ_MAX, limit_.DQ_MAX, 12);
        uint16_t t_u  =f2u(tau,-limit_.TAU_MAX,limit_.TAU_MAX,12);

        uint8_t payload_tx_[8] = {0};
        payload_tx_[0] = uint8_t(p_u>>8);
        payload_tx_[1] = uint8_t(p_u);
        payload_tx_[2] = uint8_t(v_u>>4);
        payload_tx_[3] = uint8_t((v_u&0xf)<<4|(kp_u>>8));
        payload_tx_[4] = uint8_t(kp_u);
        payload_tx_[5] = uint8_t(kd_u>>4);
        payload_tx_[6] = uint8_t((kd_u&0xf)<<4|(t_u>>8));
        payload_tx_[7] = uint8_t(t_u);

        canSend(can_id_, payload_tx_);
    }

    void controlVelocity(float vel)  {
        uint8_t payload_tx_[4] = {0};
        std::memcpy(payload_tx_,&vel,4);
        canSend(can_id_, payload_tx_);
    }

    void controlPositionVelocity(float pos,float vel)  {
        uint8_t payload_tx_[8] = {0};
        std::memcpy(payload_tx_, &pos, 4); 
        std::memcpy(payload_tx_+4, &vel, 4);
        canSend(can_id_, payload_tx_);
    }

    const MotorState& getState()     const { return state_; }
    uint32_t          getMotorId()   const { return motor_id_; }
    uint32_t          getCanId()     const { return can_id_; }
    uint32_t          getMasterId()  const { return master_id_; }

    void canSend(uint32_t frame_id, uint8_t* data)  {
        if(dm_device_ != NULL){
            device_channel_send_fast(dm_device_, 0, frame_id, 1, false, false, false, 8, data);
        }
    };
    void canRecv(const uint8_t* data)  {

        using namespace utils;
        uint16_t q_u  =(uint16_t(data[1])<<8)|data[2];
        uint16_t dq_u =(uint16_t(data[3])<<4)|(data[4]>>4);
        uint16_t tau_u=(uint16_t(data[4]&0xf)<<8)|data[5];

        state_.position  = u2f(q_u,  -limit_.Q_MAX,  limit_.Q_MAX,  16);
        state_.velocity  = u2f(dq_u, -limit_.DQ_MAX, limit_.DQ_MAX, 12);
        state_.torque    = u2f(tau_u,-limit_.TAU_MAX,limit_.TAU_MAX,12);
        state_.temperate = data[6];
        state_.online    = true;
    }

private:
    device_handle* dm_device_;

    uint32_t    motor_id_;    // motor id
    uint32_t    can_id_;      // can frame tx id
    uint32_t    master_id_;   // can frame rx id

    MotorState  state_;       // motor state
    ModeOffset  mode_;

    LimitParam  limit_;
};

} // namespace damiao