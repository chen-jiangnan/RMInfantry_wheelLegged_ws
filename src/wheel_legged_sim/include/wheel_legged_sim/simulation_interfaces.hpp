# pragma once

#include <cmath>
#include <cstdint>
#include <mutex> 

class Quaternion
{
public:
    Quaternion(){}
    double* quaternion(){ return quaternion_; }
private:
    double quaternion_[4];
};

class RPY
{
public:
    RPY(){}
    double* rpy(){ return rpy_; }
private:
    double rpy_[3];
};

class Gyroscope
{
public:
    Gyroscope(){}
    double* gyroscope(){ return gyroscope_; }
private:
    double gyroscope_[3];
};

class Accelerometer
{
public:
    Accelerometer(){}
    double* accelerometer(){ return accelerometer_; }
private:
    double accelerometer_[3];
};

class IMUState_t 
{
public:
    IMUState_t() {
        // 初始化所有数据为0
        for(int i = 0; i < 4; i++) quaternion_[i] = 0.0;
        for(int i = 0; i < 3; i++) {
            rpy_[i] = 0.0;
            gyroscope_[i] = 0.0;
            accelerometer_[i] = 0.0;
        }
    }
    
    // 获取四元数数据指针
    double* quaternion() { return quaternion_; }
    
    // 获取RPY数据指针
    double* rpy() { return rpy_; }
    
    // 获取陀螺仪数据指针
    double* gyroscope() { return gyroscope_; }
    
    // 获取加速度计数据指针
    double* accelerometer() { return accelerometer_; }
    
    // 从MuJoCo传感器数据更新IMU状态
    // quat_adr: 四元数传感器地址，如果<0则跳过
    // gyro_adr: 陀螺仪传感器地址，如果<0则跳过
    // acc_adr: 加速度计传感器地址，如果<0则跳过
    // sensordata: MuJoCo传感器数据数组
    void updateFromMuJoCo(int quat_adr, int gyro_adr, int acc_adr, const double* sensordata) {
        // 更新四元数
        if(quat_adr >= 0) {
            quaternion_[0] = sensordata[quat_adr + 0];
            quaternion_[1] = sensordata[quat_adr + 1];
            quaternion_[2] = sensordata[quat_adr + 2];
            quaternion_[3] = sensordata[quat_adr + 3];
            
            // 从四元数计算RPY
            double w = quaternion_[0];
            double x = quaternion_[1];
            double y = quaternion_[2];
            double z = quaternion_[3];
            
            rpy_[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
            rpy_[1] = asin(2 * (w * y - z * x));
            rpy_[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        }
        
        // 更新陀螺仪数据
        if(gyro_adr >= 0) {
            gyroscope_[0] = sensordata[gyro_adr + 0];
            gyroscope_[1] = sensordata[gyro_adr + 1];
            gyroscope_[2] = sensordata[gyro_adr + 2];
        }
        
        // 更新加速度计数据
        if(acc_adr >= 0) {
            accelerometer_[0] = sensordata[acc_adr + 0];
            accelerometer_[1] = sensordata[acc_adr + 1];
            accelerometer_[2] = sensordata[acc_adr + 2];
        }
    }
    
private:
    double quaternion_[4];      // 四元数 [w, x, y, z]
    double rpy_[3];             // 欧拉角 [roll, pitch, yaw]
    double gyroscope_[3];       // 陀螺仪 [x, y, z]
    double accelerometer_[3];   // 加速度计 [x, y, z]
};



class MotorCmd_t
{
public:
    MotorCmd_t() : mode_(0), q_(0), dq_(0), tau_(0), p_kp_(0), p_kd_(0), v_kp_(0), v_kd_(0) {}
    
    uint8_t& mode() { return mode_; }
    double& q() { return q_; }
    double& dq() { return dq_; }
    double& tau() { return tau_; }
    double& p_kp() { return p_kp_; }
    double& p_kd() { return p_kd_; }
    double& v_kp() { return v_kp_; }
    double& v_kd() { return v_kd_; }
    
private:
    uint8_t mode_;
    double q_;
    double dq_;
    double tau_;
    double p_kp_;
    double p_kd_;
    double v_kp_;
    double v_kd_;
};

class MotorState_t
{
public:
    MotorState_t() : mode_(0), q_(0), dq_(0), tau_est_(0) {}
    
    uint8_t& mode() { return mode_; }
    double& q() { return q_; }
    double& dq() { return dq_; }
    double& tau_est() { return tau_est_; }
    
private:
    uint8_t mode_;
    double q_;
    double dq_;
    double tau_est_;
};

class LowState_t
{
public:
    LowState_t() : locked_(false) {}
    
    MotorState_t* motor_state(){return motor_state_;}
    IMUState_t* chassis_imu_state(){return imu_state_;}
    IMUState_t* gimbal_imu_state(){return &imu_state_[1];}
    
    // 尝试获取锁（非阻塞）
    bool trylock() {
        if (mutex_.try_lock()) {
            locked_ = true;
            return true;
        }
        return false;
    }
    
    // 解锁并发布（这里只是解锁，实际发布在ROS2节点中完成）
    void unlock() {
        if (locked_) {
            mutex_.unlock();
            locked_ = false;
        }
    }
    int iflocked(){
        if(locked_ == true){
            return 1;
        }else{
            return 0;
        }
    }

private:
    std::mutex mutex_;
    bool locked_;
    MotorState_t motor_state_[11];
    IMUState_t imu_state_[2];
};

class LowCmd_t
{
public:
    LowCmd_t(){}
    MotorCmd_t* motor_state(){return motor_state_;}
    
    // 互斥锁，用于保护多线程访问
    std::mutex mutex_;
    
private:
    MotorCmd_t motor_state_[11];
};

class HighState_t
{
public:
    HighState_t(){}
    double* position(){return position_;}
    double* velocity(){return velocity_;}
    // 尝试获取锁（非阻塞）
    bool trylock() {
        if (mutex_.try_lock()) {
            locked_ = true;
            return true;
        }
        return false;
    }
    
    // 解锁并发布（这里只是解锁，实际发布在ROS2节点中完成）
    void unlock() {
        if (locked_) {
            mutex_.unlock();
            locked_ = false;
        }
    }

private:
    double position_[3];
    double velocity_[3];
    // 互斥锁，用于保护多线程访问
    std::mutex mutex_;
    bool locked_;
};