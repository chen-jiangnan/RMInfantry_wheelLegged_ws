#pragma once

#include <cmath>
#include <cstdint>
#include <mutex>
#include <atomic>
#include <array>

/* ================================================================
 *  SimMotorCmd：单电机指令（替代 MotorCmd_t）
 *  SimMotorState：单电机状态（替代 MotorState_t）
 *
 *  使用 std::atomic<double> 保证无锁线程安全：
 *   - ROS 回调线程写 cmd
 *   - MuJoCo 物理线程读 cmd / 写 state
 *   - ROS timer 线程读 state
 * ================================================================ */

// static constexpr int kNumChassisMotors = 6;
// static constexpr int kNumGimbalMotors  = 2;
// static constexpr int kNumShootMotors   = 3;
// static constexpr int kNumMotors        = kNumChassisMotors + kNumGimbalMotors + kNumShootMotors;
// kNumMotors 由 RobotBridge 从 m->nu 读取
static constexpr int kMaxMotors = 16;  // 上限，用于数组大

struct SimMotorCmd {
    std::atomic<uint8_t> mode{0};
    std::atomic<double>  q{0.0};
    std::atomic<double>  dq{0.0};
    std::atomic<double>  tau{0.0};
    std::atomic<double>  kp{0.0};
    std::atomic<double>  kd{0.0};

    // atomic 不可拷贝，提供显式赋值
    void set(uint8_t m, double q_, double dq_, double tau_, double kp_, double kd_) {
        mode.store(m,   std::memory_order_relaxed);
        q.store(q_,     std::memory_order_relaxed);
        dq.store(dq_,   std::memory_order_relaxed);
        tau.store(tau_, std::memory_order_relaxed);
        kp.store(kp_,   std::memory_order_relaxed);
        kd.store(kd_,   std::memory_order_relaxed);
    }
};

struct SimMotorState {
    std::atomic<uint8_t> mode{0};
    std::atomic<double>  q{0.0};
    std::atomic<double>  dq{0.0};
    std::atomic<double>  tau{0.0};

    void set(double q_, double dq_, double tau_) {
        q.store(q_,     std::memory_order_relaxed);
        dq.store(dq_,   std::memory_order_relaxed);
        tau.store(tau_, std::memory_order_relaxed);
    }
};

/* ================================================================
 *  SimIMUState：IMU 状态（线程安全读写）
 * ================================================================ */
struct SimIMUState {
    std::mutex mtx;
    double quaternion[4]    = {1, 0, 0, 0};
    double rpy[3]           = {0, 0, 0};
    double gyroscope[3]     = {0, 0, 0};
    double accelerometer[3] = {0, 0, 0};

    void updateFromMuJoCo(int quat_adr, int gyro_adr, int acc_adr,
                          const double* sensordata)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (quat_adr >= 0) {
            for (int i = 0; i < 4; ++i)
                quaternion[i] = sensordata[quat_adr + i];

            double w = quaternion[0], x = quaternion[1],
                   y = quaternion[2], z = quaternion[3];
            rpy[0] = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
            rpy[1] = asin (2*(w*y - z*x));
            rpy[2] = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
        }
        if (gyro_adr >= 0)
            for (int i = 0; i < 3; ++i) gyroscope[i]     = sensordata[gyro_adr + i];
        if (acc_adr >= 0)
            for (int i = 0; i < 3; ++i) accelerometer[i] = sensordata[acc_adr  + i];
    }

    // 读取时加锁拷贝
    void read(double out_quat[4], double out_gyro[3],
              double out_acc[3],  double out_rpy[3]) const
    {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(mtx));
        for (int i = 0; i < 4; ++i) out_quat[i] = quaternion[i];
        for (int i = 0; i < 3; ++i) {
            out_gyro[i] = gyroscope[i];
            out_acc[i]  = accelerometer[i];
            out_rpy[i]  = rpy[i];
        }
    }
};

/* ================================================================
 *  SimSharedData：所有线程共享的仿真数据
 *
 *  电机 cmd/state 用 atomic 无锁
 *  IMU 用 mutex（数据量大，原子操作不合适）
 * ================================================================ */
struct SimSharedData {
    SimMotorCmd   motor_cmd[kMaxMotors];
    SimMotorState motor_state[kMaxMotors];
    SimIMUState   chassis_imu;
    SimIMUState   gimbal_imu;

    std::atomic<int> num_motors{0};        // 实际电机数，由 RobotBridge 设置
    std::atomic<int> num_chassis{0};
    std::atomic<int> num_gimbal{0};
    std::atomic<int> num_shoot{0};

    // 位置速度（底盘）
    std::atomic<double> pos_x{0}, pos_y{0}, pos_z{0};
    std::atomic<double> vel_x{0}, vel_y{0}, vel_z{0};
};