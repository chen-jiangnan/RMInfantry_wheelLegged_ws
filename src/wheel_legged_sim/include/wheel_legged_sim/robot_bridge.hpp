#pragma once

#include <mujoco/mujoco.h>
#include <iostream>
#include <cstring>

#include "simulation_interfaces.hpp"
#include "param.h"

#define MOTOR_SENSOR_NUM 3

/* ================================================================
 *  RobotBridge
 *
 *  职责：在 MuJoCo 物理线程里运行
 *   - run()：从 shared_.motor_cmd 读取指令写入 mjData->ctrl
 *            从 mjData->sensordata 读取状态写入 shared_.motor_state
 *            更新 IMU 状态
 *
 *  线程安全：
 *   - motor_cmd/state 用 atomic，无锁
 *   - IMU 用内部 mutex
 * ================================================================ */
class RobotBridge {
public:
    RobotBridge(mjModel* model, mjData* data, SimSharedData& shared)
        : m_(model), d_(data), shared_(shared)
    {
        // 从 actuator 名称推断分组
        int n_chassis = 0, n_gimbal = 0, n_shoot = 0;
        for (int i = 0; i < m_->nu; ++i) {
            const char* name = mj_id2name(m_, mjOBJ_ACTUATOR, i);
            if (!name) continue;
            std::string s(name);
            if (s.find("gimbal") != std::string::npos)       n_gimbal++;
            else if (s.find("friction") != std::string::npos ||
                     s.find("feeder")   != std::string::npos) n_shoot++;
            else                                              n_chassis++;
        }

        shared_.num_motors.store(m_->nu);
        shared_.num_chassis.store(n_chassis);
        shared_.num_gimbal.store(n_gimbal);
        shared_.num_shoot.store(n_shoot);

        num_motor_ = m_->nu;

        printf("Motors: total=%d chassis=%d gimbal=%d shoot=%d\n",
            m_->nu, n_chassis, n_gimbal, n_shoot);
        
        findSensors();

        if (param::config.print_scene_information)
            printSceneInfo();
    }

    void run() {
        if (!d_) return;
        /* ── 写控制量到 MuJoCo ── */
        for (int i = 0; i < num_motor_; ++i) {
            auto& cmd = shared_.motor_cmd[i];
            d_->ctrl[i] = cmd.tau.load(std::memory_order_relaxed)
                        + cmd.kp.load(std::memory_order_relaxed)
                          * (cmd.q.load(std::memory_order_relaxed)
                             - d_->sensordata[i])
                        + cmd.kd.load(std::memory_order_relaxed)
                          * (cmd.dq.load(std::memory_order_relaxed)
                             - d_->sensordata[i + num_motor_]);
        }

        /* ── 读传感器到 shared state ── */
        for (int i = 0; i < num_motor_; ++i) {
            shared_.motor_state[i].set(
                d_->sensordata[i],
                d_->sensordata[i + num_motor_],
                d_->sensordata[i + 2 * num_motor_]);
        }

        /* ── IMU ── */
        shared_.chassis_imu.updateFromMuJoCo(
            chassis_imu_quat_adr_, chassis_imu_gyro_adr_, chassis_imu_acc_adr_,
            d_->sensordata);

        shared_.gimbal_imu.updateFromMuJoCo(
            gimbal_imu_quat_adr_, gimbal_imu_gyro_adr_, gimbal_imu_acc_adr_,
            d_->sensordata);

        /* ── 位置速度 ── */
        if (frame_pos_adr_ >= 0) {
            shared_.pos_x.store(d_->sensordata[frame_pos_adr_],     std::memory_order_relaxed);
            shared_.pos_y.store(d_->sensordata[frame_pos_adr_ + 1], std::memory_order_relaxed);
            shared_.pos_z.store(d_->sensordata[frame_pos_adr_ + 2], std::memory_order_relaxed);
        }
        if (frame_vel_adr_ >= 0) {
            shared_.vel_x.store(d_->sensordata[frame_vel_adr_],     std::memory_order_relaxed);
            shared_.vel_y.store(d_->sensordata[frame_vel_adr_ + 1], std::memory_order_relaxed);
            shared_.vel_z.store(d_->sensordata[frame_vel_adr_ + 2], std::memory_order_relaxed);
        }
    }

private:
    void findSensors() {
        auto find = [&](const char* name) -> int {
            int id = mj_name2id(m_, mjOBJ_SENSOR, name);
            return id >= 0 ? m_->sensor_adr[id] : -1;
        };
        chassis_imu_quat_adr_ = find("chassis_imu_quat");
        chassis_imu_gyro_adr_ = find("chassis_imu_gyro");
        chassis_imu_acc_adr_  = find("chassis_imu_acc");
        gimbal_imu_quat_adr_  = find("gimbal_imu_quat");
        gimbal_imu_gyro_adr_  = find("gimbal_imu_gyro");
        gimbal_imu_acc_adr_   = find("gimbal_imu_acc");
        frame_pos_adr_        = find("frame_pos");
        frame_vel_adr_        = find("frame_vel");
    }

    void printSceneInfo() {
        std::cout << "=== Bodies ===" << std::endl;
        for (int i = 0; i < m_->nbody; ++i) {
            const char* name = mj_id2name(m_, mjOBJ_BODY, i);
            if (name) std::cout << "  [" << i << "] " << name << std::endl;
        }
        std::cout << "=== Actuators ===" << std::endl;
        for (int i = 0; i < m_->nu; ++i) {
            const char* name = mj_id2name(m_, mjOBJ_ACTUATOR, i);
            if (name) std::cout << "  [" << i << "] " << name << std::endl;
        }
        int adr = 0;
        std::cout << "=== Sensors ===" << std::endl;
        for (int i = 0; i < m_->nsensor; ++i) {
            const char* name = mj_id2name(m_, mjOBJ_SENSOR, i);
            if (name) std::cout << "  [" << adr << "] " << name
                                << " dim=" << m_->sensor_dim[i] << std::endl;
            adr += m_->sensor_dim[i];
        }
    }

    mjModel*        m_;
    mjData*         d_;
    SimSharedData&  shared_;
    int             num_motor_ = 0;

    int chassis_imu_quat_adr_ = -1, chassis_imu_gyro_adr_ = -1, chassis_imu_acc_adr_ = -1;
    int gimbal_imu_quat_adr_  = -1, gimbal_imu_gyro_adr_  = -1, gimbal_imu_acc_adr_  = -1;
    int frame_pos_adr_ = -1, frame_vel_adr_ = -1;
};