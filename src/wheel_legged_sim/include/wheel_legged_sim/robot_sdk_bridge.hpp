# pragma once
#include <mujoco/mujoco.h>
#include "wheel_legged_msgs/msg/chassis_joint_cmd.hpp"
#include "wheel_legged_msgs/msg/chassis_joint_state.hpp"
#include "wheel_legged_msgs/msg/gimbal_joint_cmd.hpp"
#include "wheel_legged_msgs/msg/gimbal_joint_state.hpp"
#include "wheel_legged_msgs/msg/shoot_joint_cmd.hpp"
#include "wheel_legged_msgs/msg/shoot_joint_state.hpp"
#include "wheel_legged_msgs/msg/imu_state.hpp"

#include <iostream>
#include <rclcpp/logging.hpp>

#include "wheel_legged_sim/param.h"
#include "wheel_legged_sim/simulation_interfaces.hpp"
// #include "physics_joystick.h"

#define MOTOR_SENSOR_NUM 3

class RobotSDKBridgeBase
{
public:
    RobotSDKBridgeBase(mjModel *model, mjData *data)
    : mj_model_(model), mj_data_(data)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        // if(param::config.use_joystick == 1) {
        //     if(param::config.joystick_type == "xbox") {
        //         joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
        //     } else if(param::config.joystick_type == "switch") {
        //         joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
        //     } else {
        //         std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
        //         exit(EXIT_FAILURE);
        //     }
        // }

    }

    virtual void start() {}

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjModel *mj_model_;
    mjData *mj_data_;

    // Sensor data indices
    int chassis_imu_quat_adr_ = -1;
    int chassis_imu_gyro_adr_ = -1;
    int chassis_imu_acc_adr_ = -1;

    int gimbal_imu_quat_adr_ = -1;
    int gimbal_imu_gyro_adr_ = -1;
    int gimbal_imu_acc_adr_ = -1;

    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;
    // std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

    void _check_sensor()
    {
        printf("test: _check_sensor begin");
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;
    
        // Find sensor addresses by name
        int sensor_id = -1;
        
        // IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "chassis_imu_quat");
        if (sensor_id >= 0) {
            chassis_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "chassis_imu_gyro");
        if (sensor_id >= 0) {
            chassis_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "chassis_imu_acc");
        if (sensor_id >= 0) {
            chassis_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame position
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
        if (sensor_id >= 0) {
            frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame velocity
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
        if (sensor_id >= 0) {
            frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Gimabl IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "gimbal_imu_quat");
        if (sensor_id >= 0) {
            gimbal_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Gimabl IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "gimbal_imu_gyro");
        if (sensor_id >= 0) {
            gimbal_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Gimabl IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "gimbal_imu_acc");
        if (sensor_id >= 0) {
            gimbal_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        printf("test: _check_sensor end");
    }
};

class RobotBridge : public RobotSDKBridgeBase
{
public:
    RobotBridge(mjModel *model, mjData *data) : RobotSDKBridgeBase(model, data)
    {
        lowcmd = std::make_shared<LowCmd_t>();
        lowstate = std::make_unique<LowState_t>();
        highstate = std::make_unique<HighState_t>();
    }

    virtual void run()
    {
        if(!mj_data_) return;
        // lowcmd: 从 lowcmd 读取数据写入 mjdata
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
        }

        // lowstate: 从 mjdata 读取数据写入 lowstate
        if(lowstate->trylock()) {
            for(int i(0); i<num_motor_; i++) {
                lowstate->motor_state()[i].q() = mj_data_->sensordata[i];
                lowstate->motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                lowstate->motor_state()[i].tau() = mj_data_->sensordata[i + 2 * num_motor_];
            }
            
            // 更新底盘IMU状态
            if(chassis_imu_quat_adr_ >= 0) {
                lowstate->chassis_imu_state()->updateFromMuJoCo(
                    chassis_imu_quat_adr_,
                    chassis_imu_gyro_adr_,
                    chassis_imu_acc_adr_,
                    mj_data_->sensordata
                );
            }
            
            lowstate->unlock();
            // printf("lowstate unlock\n");
        }
        // highstate
        if(highstate->trylock()) {
            if(frame_pos_adr_ >= 0) {
                highstate->position()[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
                highstate->position()[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
                highstate->position()[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
            }
            if(frame_vel_adr_ >= 0) {
                highstate->velocity()[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
                highstate->velocity()[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
                highstate->velocity()[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
            }
            highstate->unlock();
            
        }
    }

    std::unique_ptr<HighState_t> highstate;
    std::unique_ptr<LowState_t> lowstate;
    std::shared_ptr<LowCmd_t> lowcmd;
};
