/**
 * @file hipnuc_imu_adapted.hpp
 * @brief 基于官方代码改造的非阻塞IMU驱动
 */

#pragma once

#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>

extern "C" {
#include "hipnuc_dec.h"
}

namespace hipnuc_imu {

#define GRA_ACC     (9.8)
#define DEG_TO_RAD  (0.01745329)
#define BUF_SIZE    (1024)

// IMU数据结构
struct HipnucData {
    // 四元数
    float quat[4];  // w, x, y, z
    
    // 角速度 (rad/s)
    float gyro[3];  // x, y, z
    
    // 加速度 (m/s^2)
    float acc[3];   // x, y, z
    
    // 欧拉角 (rad) 
    float rpy[3];   // roll, pitch, yaw
    
    // 时间戳
    uint64_t timestamp_us;
    
    // 温度
    float temperature;
    
    // 协议类型（用于调试）
    uint8_t protocol_tag;  // 0x91, 0x83, etc.
};

class HipnucImu{
public:
    explicit HipnucImu(const std::string& port = "/dev/ttyIMU",
                            int baudrate = 921600)
        : port_(port), baudrate_(baudrate), fd_(-1) {
        memset(&raw_, 0, sizeof(raw_));
    }
    
    ~HipnucImu() {
        stop();
        if (fd_ >= 0) {
            close(fd_);
        }
    }
    
    bool init() {
        fd_ = open_ttyport(port_, baudrate_);
        if (fd_ < 0) {
            last_error_ = "无法打开串口: " + port_;
            return false;
        }
        return true;
    }
    
    void start() {
        if (running_) return;
        running_ = true;
        recv_thread_ = std::thread(&HipnucImu::receiveLoop, this);
    }
    
    void stop() {
        if (!running_) return;
        running_ = false;
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
    }
    
    void setDataCallback(std::function<void(const HipnucData&)> callback) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        data_callback_ = callback;
    }
    
    bool getLatestData(HipnucData& data) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (latest_data_.timestamp_us == 0) {
            return false;
        }
        data = latest_data_;
        return true;
    }
    
    bool isOnline(int timeout_ms = 100) const {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_update_time_).count();
        return elapsed < timeout_ms;
    }
    
    std::string getLastError() const { return last_error_; }

private:
    // ==================== 官方串口配置（直接复用）====================
    int open_ttyport(const std::string& tty_port, int baud) {
        const char *port_device = tty_port.c_str();
        int serial_port = open(port_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_port < 0) {
            perror("Error opening serial port");
            return -1;
        }

        struct termios2 tty;

        if (ioctl(serial_port, TCGETS2, &tty) != 0) {
            perror("Error from TCGETS2 ioctl");
            close(serial_port);
            return -1;
        }

        tty.c_cflag &= ~CBAUD;
        tty.c_cflag |= BOTHER;

        tty.c_ispeed = baud;
        tty.c_ospeed = baud;

        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        tty.c_cc[VTIME] = 10;
        tty.c_cc[VMIN] = 0;

        if (ioctl(serial_port, TCSETS2, &tty) != 0) {
            perror("Error from TCSETS2 IOCTL");
            close(serial_port);
            return -1;
        }

        return serial_port;
    }
    
    // ==================== 非阻塞接收循环 ====================
    void receiveLoop() {
        uint8_t buf[BUF_SIZE];
        struct pollfd p;
        p.fd = fd_;
        p.events = POLLIN;
        
        while (running_) {
            int rpoll = poll(&p, 1, 10);  // 10ms超时
            
            if (rpoll == 0) {
                // 超时，继续循环
                continue;
            }
            
            if (rpoll < 0) {
                // 错误
                break;
            }
            
            int n = read(fd_, buf, sizeof(buf));
            
            if (n > 0) {
                parseData(buf, n);
            }
        }
    }
    
    // ==================== 官方解析逻辑（直接复用）====================
    void parseData(const uint8_t* buf, size_t n) {
        for (size_t i = 0; i < n; i++) {
            int rev = hipnuc_input(&raw_, buf[i]);
            
            if (rev) {
                HipnucData data;
                memset(&data, 0, sizeof(data));
                
                // 0x83协议（推荐，bitmap灵活配置）
                if (raw_.hi83.tag == 0x83) {
                    data.protocol_tag = 0x83;
                    uint32_t bm = raw_.hi83.data_bitmap;
                    
                    if (bm & HI83_BMAP_QUAT) {
                        data.quat[0] = raw_.hi83.quat[0];
                        data.quat[1] = raw_.hi83.quat[1];
                        data.quat[2] = raw_.hi83.quat[2];
                        data.quat[3] = raw_.hi83.quat[3];
                    }
                    
                    if (bm & HI83_BMAP_GYR_B) {
                        data.gyro[0] = raw_.hi83.gyr_b[0];
                        data.gyro[1] = raw_.hi83.gyr_b[1];
                        data.gyro[2] = raw_.hi83.gyr_b[2];
                    }
                    
                    if (bm & HI83_BMAP_ACC_B) {
                        data.acc[0] = raw_.hi83.acc_b[0];
                        data.acc[1] = raw_.hi83.acc_b[1];
                        data.acc[2] = raw_.hi83.acc_b[2];
                    }
                    
                    if (bm & HI83_BMAP_RPY) {
                        data.rpy[0] = raw_.hi83.rpy[0];
                        data.rpy[1] = raw_.hi83.rpy[1];
                        data.rpy[2] = raw_.hi83.rpy[2];
                    }
                    
                    // if (bm & HI83_BMAP_MAG_B) {
                    //     data.mag[0] = raw_.hi83.mag_b[0];
                    //     data.mag[1] = raw_.hi83.mag_b[1];
                    //     data.mag[2] = raw_.hi83.mag_b[2];
                    // }
                    
                    if (bm & HI83_BMAP_TEMPERATURE) {
                        data.temperature = raw_.hi83.temperature;
                    }
                }
                // 0x91协议（旧版，固定格式）
                else if (raw_.hi91.tag == 0x91) {
                    data.protocol_tag = 0x91;
                    
                    data.quat[0] = raw_.hi91.quat[0];
                    data.quat[1] = raw_.hi91.quat[1];
                    data.quat[2] = raw_.hi91.quat[2];
                    data.quat[3] = raw_.hi91.quat[3];
                    
                    data.gyro[0] = raw_.hi91.gyr[0] * DEG_TO_RAD;
                    data.gyro[1] = raw_.hi91.gyr[1] * DEG_TO_RAD;
                    data.gyro[2] = raw_.hi91.gyr[2] * DEG_TO_RAD;
                    
                    data.acc[0] = raw_.hi91.acc[0] * GRA_ACC;
                    data.acc[1] = raw_.hi91.acc[1] * GRA_ACC;
                    data.acc[2] = raw_.hi91.acc[2] * GRA_ACC;
                    
                    // data.mag[0] = raw_.hi91.mag[0];
                    // data.mag[1] = raw_.hi91.mag[1];
                    // data.mag[2] = raw_.hi91.mag[2];
                    
                    data.rpy[0] = raw_.hi91.roll * DEG_TO_RAD;
                    data.rpy[1] = raw_.hi91.pitch * DEG_TO_RAD;
                    data.rpy[2] = raw_.hi91.yaw * DEG_TO_RAD;
                    
                    data.temperature = raw_.hi91.temp;
                }
                
                // 时间戳
                data.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                
                // 更新缓存
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    latest_data_ = data;
                    last_update_time_ = std::chrono::steady_clock::now();
                }
                
                // 触发回调
                {
                    std::lock_guard<std::mutex> lock(callback_mutex_);
                    if (data_callback_) {
                        data_callback_(data);
                    }
                }
            }
        }
    }
    
    std::string port_;
    int baudrate_;
    int fd_;
    
    hipnuc_raw_t raw_;  // 官方解码器状态
    
    std::thread recv_thread_;
    std::atomic<bool> running_{false};
    
    HipnucData latest_data_;
    std::mutex data_mutex_;
    std::chrono::steady_clock::time_point last_update_time_;
    
    std::function<void(const HipnucData&)> data_callback_;
    std::mutex callback_mutex_;
    
    std::string last_error_;
};

}
