#pragma once
#include <array>
#include <cstdint>
#include <unordered_map>
#include <cstring>
#include <cmath>
extern "C" {
    #include "pub_user.h"
}

// enum DM_Motor_Type : uint8_t {
//     M3508=0, M2006, M6020, Num_Of_Motor
// };
// struct LimitParam {uint16_t I_MAX, V_MAX;};
// static constexpr LimitParam kLimitTable[Num_Of_Motor] = {
//     {16384, 0},         // M3508          check 
//     {10000, 0},         // M2006          check
//     {16384, 25000},     // M6020          check
// };


namespace dji {

constexpr float TWO_PI = 6.28318530717958647692f;
constexpr float ECD_RESOLUTION = 8192.0f;


enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};
struct pid_type_def
{
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次
    
    float integral_valve;//积分阀门

};

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


class M3508Motor{
public:
    M3508Motor(){}

    struct motor_measure_t
    {
        uint16_t ecd;			//机械转角
        int16_t speed_rpm;  	//电机转速
        int16_t given_current;	//扭矩电流
        uint8_t temperate;		//温度
    };

    /* -------- 初始化 -------- */
    void enable(uint16_t motor_id,
        PID_MODE mode, float kp, float ki, float kd, float iout_max, float out_max){
        motor_id_ = motor_id;
        master_id_ = 0x200 + motor_id;
        if(motor_id_ <= 4){can_id_ = 0x200;}
        else{can_id_ = 0x1FF;}
        setPIDMode(mode);
        setPIDParams(kp, ki, kd, iout_max, out_max);
    };
    
    void disable(){
        pid_ctrl_.out = 0.0f;
        pid_ctrl_.Pout = 0.0f;
        pid_ctrl_.Iout = 0.0f;
        pid_ctrl_.Dout = 0.0f;
        pid_ctrl_.error[0] = pid_ctrl_.error[1] = pid_ctrl_.error[2] = 0.0f;
        pid_ctrl_.Dbuf[0]  = pid_ctrl_.Dbuf[1]  = pid_ctrl_.Dbuf[2]  = 0.0f;
    }

    /* -------- 设置参数 ------ */
    void setPIDMode(PID_MODE mode){
        pid_ctrl_.mode = mode;
    }
    void setPIDParams(float kp, float ki, float kd){
        pid_ctrl_.Kp = kp;
        pid_ctrl_.Ki = ki;
        pid_ctrl_.Kd = kd;

    }
    void setPIDParams(float kp, float ki, float kd, float max_iout, float max_out){
        pid_ctrl_.Kp = kp;
        pid_ctrl_.Ki = ki;
        pid_ctrl_.Kd = kd;
        pid_ctrl_.max_iout = max_iout;
        pid_ctrl_.max_out = max_out;
    }

    /* -------- 控制 -------- */
    int16_t controlVelocity(int16_t vel){
        pid_ctrl_.error[2] = pid_ctrl_.error[1];
        pid_ctrl_.error[1] = pid_ctrl_.error[0];
        pid_ctrl_.set = vel;
        pid_ctrl_.fdb = motor_state_.speed_rpm;
        pid_ctrl_.error[0] = pid_ctrl_.set - pid_ctrl_.fdb;

        if (pid_ctrl_.mode == PID_POSITION)
        {
            pid_ctrl_.Pout = pid_ctrl_.Kp * pid_ctrl_.error[0];
            pid_ctrl_.Iout += pid_ctrl_.Ki * pid_ctrl_.error[0];
            pid_ctrl_.Dbuf[2] = pid_ctrl_.Dbuf[1];
            pid_ctrl_.Dbuf[1] = pid_ctrl_.Dbuf[0];
            pid_ctrl_.Dbuf[0] = (pid_ctrl_.error[0] - pid_ctrl_.error[1]);
            pid_ctrl_.Dout = pid_ctrl_.Kd * pid_ctrl_.Dbuf[0];
            LimitMax(pid_ctrl_.Iout, pid_ctrl_.max_iout);
            pid_ctrl_.out = pid_ctrl_.Pout + pid_ctrl_.Iout + pid_ctrl_.Dout;
            LimitMax(pid_ctrl_.out, pid_ctrl_.max_out);
            
            return pid_ctrl_.out;
        }
        else if (pid_ctrl_.mode == PID_DELTA)
        {
            pid_ctrl_.Pout = pid_ctrl_.Kp * (pid_ctrl_.error[0] - pid_ctrl_.error[1]);
            pid_ctrl_.Iout = pid_ctrl_.Ki * pid_ctrl_.error[0];
            pid_ctrl_.Dbuf[2] = pid_ctrl_.Dbuf[1];
            pid_ctrl_.Dbuf[1] = pid_ctrl_.Dbuf[0];
            pid_ctrl_.Dbuf[0] = (pid_ctrl_.error[0] - 2.0f * pid_ctrl_.error[1] + pid_ctrl_.error[2]);
            pid_ctrl_.Dout = pid_ctrl_.Kd * pid_ctrl_.Dbuf[0];
            pid_ctrl_.out += pid_ctrl_.Pout + pid_ctrl_.Iout + pid_ctrl_.Dout;
            LimitMax(pid_ctrl_.out, pid_ctrl_.max_out);
            
            return pid_ctrl_.out;
        }
        else
        {
            return 0;
        }
    }

    float ecd2rad(int16_t ecd){
        return static_cast<float>(ecd) * TWO_PI / ECD_RESOLUTION;
    }
    int16_t rad2ecd(float rad){
        float wrapped = std::fmod(rad, TWO_PI);
        if (wrapped < 0.0f)
            wrapped += TWO_PI;
    
        return static_cast<int16_t>(wrapped / TWO_PI * ECD_RESOLUTION);
    }

    float rpm2RadPerSec(int16_t rpm){
        return static_cast<float>(rpm) * TWO_PI / 60.0f;
    }
    int16_t radPerSec2rpm(float rad_per_sec){
        return static_cast<int16_t>(rad_per_sec * 60.0f / TWO_PI);
    }
    
    int16_t current2cmd(float current){
        if (current > 20.0f)  current = 20.0f;
        if (current < -20.0f) current = -20.0f;
        return static_cast<int16_t>(current / 20.0f * 16384.0f);
    }
    float cmd2current(int16_t cmd){
        float current = (float(cmd) / 16384.0f) * 20.0f;
        if (current > 20.0f)  current = 20.0f;
        if (current < -20.0f) current = -20.0f;
        return current;        
    }

    float current2force(float current){
        return current*0.015622389f;
    }
    float force2current(float force){
        return force/0.015622389f;
    }
    

    const motor_measure_t& getState()     const { return motor_state_; }
    uint32_t               getMotorId()   const { return motor_id_; }
    uint32_t               getCanId()     const { return can_id_; }
    uint32_t               getMasterId()  const { return master_id_; }

    /* -------- 解析电机反馈帧 -------- */
    void canRecv(const uint8_t* data){                                
        motor_state_.ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            
        motor_state_.speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      
        motor_state_.given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  
        motor_state_.temperate = (data)[6];  
    }

private:
    uint32_t    motor_id_;    // motor id
    uint32_t    can_id_;      // can frame tx id
    uint32_t    master_id_;   // can frame rx id
    motor_measure_t motor_state_;
    pid_type_def pid_ctrl_;

};


class WheelMotors{
public:
    WheelMotors(uint16_t left_wheel_id, uint16_t right_wheel_id, device_handle* dm_device)
            :   dm_device_(dm_device)
            {
                wheelMotors_[0].enable(left_wheel_id, PID_POSITION,0,0,0,0,0);
                wheelMotors_[1].enable(right_wheel_id, PID_POSITION,0,0,0,0,0);
            }
    
        /* -------- 生命周期 -------- */
        void enable() {}
        void disable(){
            uint8_t payload_tx_[8] = {0};
            canSend(wheelMotors_[0].getCanId(), payload_tx_);
        }
    
        void controlForce(float left_value, float right_value){

            float set_force[2] = {left_value, right_value};
            int16_t set_cmd[2] = {0, 0};
            for(int i = 0; i < 2; i++){
                // 输出轴力矩->转子力矩->电流值
                float set_current = wheelMotors_[i].force2current(set_force[i]/ratio_);
                set_cmd[i] = wheelMotors_[i].current2cmd(set_current);
            }

            uint8_t payload_tx_[8] = {0};
            payload_tx_[0] = uint8_t(set_cmd[0] >> 8);
            payload_tx_[1] = uint8_t(set_cmd[0]);
            payload_tx_[2] = uint8_t(set_cmd[1] >> 8);
            payload_tx_[3] = uint8_t(set_cmd[1]);

            canSend(wheelMotors_[0].getCanId(), payload_tx_);
        }

        void canSend(uint32_t frame_id, uint8_t* data)  {
            if(dm_device_ != NULL){
                device_channel_send_fast(dm_device_, 0, frame_id, 1, false, false, false, 8, data);
            }
        };
    private:
        device_handle* dm_device_;
        float ratio_ = -15.764705882f;//减速比 268/17
        std::array<M3508Motor, 2> wheelMotors_;
    };
} // namespace dji