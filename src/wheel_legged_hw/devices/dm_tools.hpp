#pragma once
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <functional>
#include <unordered_map>
#include <vector>
#include <iostream>

extern "C" {
#include "pub_user.h"
}

namespace dm_tools {

class DMTools {
public:
    using RecvCallback = std::function<void(uint32_t frame_id,
                                            const uint8_t* data,
                                            uint8_t len)>;

    /* ================================================================
     *  构造
     *
     *  @param type  DEV_USB2CANFD（单通道）/ DEV_USB2CANFD_DUAL（双通道）
     *  @param sn    序列号，用于多设备时精确匹配。
     *               传 "" 时：只有1个设备直接用；多个设备用第一个并打印警告。
     * ================================================================ */
    explicit DMTools(device_def_t type = DEV_USB2CANFD,
                     const std::string& sn = "")
    {
        handle_ = damiao_handle_create(type);
        if (!handle_)
            throw std::runtime_error("DMTools: damiao_handle_create failed");

        int found = damiao_handle_find_devices(handle_);
        if (found == 0) {
            damiao_handle_destroy(handle_);
            throw std::runtime_error("DMTools: no device found");
        }

        int cnt = 0;
        device_handle* all[16]{};
        damiao_handle_get_devices(handle_, all, &cnt);

        /* ---- 打印所有设备信息 ---- */
        std::cout << "===== DMTools: found " << cnt << " device(s) =====" << std::endl;
        for (int i = 0; i < cnt; ++i) {
            char ver[256]{}, serial[256]{};
            device_get_serial_number(all[i], serial, sizeof(serial));
            std::cout << "  device[" << i << "]: " << ver << "  sn = " << serial << std::endl;
        }

        /* ---- 选择设备 ---- */
        if (cnt == 1) {
            dev_ = all[0];
            if (!sn.empty())
                std::cout << "DMTools: only 1 device, SN match skipped" << std::endl;
        } else {
            if (sn.empty()) {
                std::cerr << "DMTools: WARNING multiple devices found, no SN provided, using [0]" << std::endl;
                dev_ = all[0];
            } else {
                dev_ = nullptr;
                for (int i = 0; i < cnt; ++i) {
                    char serial[256]{};
                    device_get_serial_number(all[i], serial, sizeof(serial));
                    if (std::string(serial) == sn) {
                        dev_ = all[i];
                        std::cout << "DMTools: matched [" << i << "] sn =" << sn << std::endl;
                        break;
                    }
                }
                if (!dev_) {
                    damiao_handle_destroy(handle_);
                    throw std::runtime_error("DMTools: no device matched SN =" + sn);
                }
            }
        }

        /* ---- 打开设备 ---- */
        if (!device_open(dev_)) {
            damiao_handle_destroy(handle_);
            throw std::runtime_error("DMTools: device_open failed");
        }
        std::cout << "DMTools: device opened!!! v = " << getVersion() << " sn =" << getSerialNumber()<< std::endl;

        instance_ = this;
        device_hook_to_rec(dev_, &DMTools::recvHook);
        device_hook_to_err(dev_, &DMTools::errHook);
    }

    ~DMTools() {
        for (uint8_t ch = 0; ch < 2; ++ch)
            if (ch_open_[ch]) device_close_channel(dev_, ch);
        device_hook_to_rec(dev_, nullptr);
        device_hook_to_err(dev_, nullptr);
        device_close(dev_);
        damiao_handle_destroy(handle_);
        instance_ = nullptr;
    }

    DMTools(const DMTools&)            = delete;
    DMTools& operator=(const DMTools&) = delete;

    /* ================================================================
     *  设备信息
     * ================================================================ */
    std::string getVersion() const {
        char buf[256]{}; device_get_version(dev_, buf, sizeof(buf)); return buf;
    }
    std::string getSerialNumber() const {
        char buf[256]{}; device_get_serial_number(dev_, buf, sizeof(buf)); return buf;
    }

    /* ================================================================
     *  通道管理
     *
     *  DEV_USB2CANFD      单通道：channel 只有 0
     *  DEV_USB2CANFD_DUAL 双通道：channel 0 和 1 各自独立
     * ================================================================ */
    void openChannel(uint8_t channel = 0,
                     int   nom_baud  = 1'000'000,   // 仲裁段
                     int   dat_baud  = 1'000'000,   // 数据段（标准CAN时与nom相同）
                     bool  is_fd     = false,       // 默认标准 CAN 2.0
                     float nom_sp    = 0.75f,
                     float dat_sp    = 0.75f)
    {
        device_channel_set_baud_with_sp(dev_, channel, is_fd,
                                        nom_baud, dat_baud, nom_sp, dat_sp);
        if (!device_open_channel(dev_, channel))
            throw std::runtime_error("DMTools: openChannel failed ch="
                                     + std::to_string(channel));
        ch_open_[channel] = true;

        device_baud_t b{};
        device_channel_get_baudrate(dev_, channel, &b);
        std::cout << "DMTools: ch" << (int)channel << " opened"
                  << "  nom = " << b.can_baudrate
                  << "  dat = " << b.canfd_baudrate
                  << "  fd = "  << is_fd << std::endl;
    }

    void closeChannel(uint8_t channel = 0) {
        device_close_channel(dev_, channel);
        ch_open_[channel] = false;
    }

    device_baud_t getBaudrate(uint8_t channel) const {
        device_baud_t b{};
        device_channel_get_baudrate(dev_, channel, &b);
        return b;
    }

    /* ================================================================
     *  CAN 发送
     *
     *  默认参数 = 标准 CAN（非扩展帧，非CANFD）
     *
     *  device_channel_send_fast 签名（pub_user.h）：
     *    void device_channel_send_fast(
     *        device_handle* dev,
     *        uint8_t  ch,
     *        uint32_t can_id,
     *        int32_t  cnt,      ← 发送次数，固定填 1
     *        bool     ext,      ← 扩展帧（29位ID）
     *        bool     canfd,    ← CANFD 帧
     *        bool     brs,      ← 波特率切换（仅CANFD有意义）
     *        uint8_t  len,
     *        uint8_t* payload);
     * ================================================================ */
    void send(uint32_t frame_id,
              uint8_t* data,
              uint8_t  len,
              uint8_t  channel = 0,
              bool     ext   = false,   // 标准帧
              bool     canfd = false,   // 标准 CAN
              bool     brs   = false)
    {
        device_channel_send_fast(dev_, channel, frame_id,
                                 /*cnt=*/1,
                                 ext, canfd, brs,
                                 len, data);
    }

    /* ================================================================
     *  接收路由（三级优先级）
     * ================================================================ */
    void addRecvRoute(uint32_t frame_id, RecvCallback cb) {
        exact_map_[frame_id] = std::move(cb);
    }
    void removeRecvRoute(uint32_t frame_id) {
        exact_map_.erase(frame_id);
    }
    void addRecvRange(uint32_t id_min, uint32_t id_max, RecvCallback cb) {
        ranges_.push_back({id_min, id_max, std::move(cb)});
    }
    void setDefaultRecvCallback(RecvCallback cb) {
        default_cb_ = std::move(cb);
    }

private:
    damiao_handle* handle_ = nullptr;
    device_handle* dev_    = nullptr;
    bool           ch_open_[2]{};

    std::unordered_map<uint32_t, RecvCallback> exact_map_;
    struct Range { uint32_t min, max; RecvCallback cb; };
    std::vector<Range> ranges_;
    RecvCallback default_cb_;

    static DMTools* instance_;

    static void recvHook(usb_rx_frame_t* frame) {
        if (!frame || !instance_) return;
        instance_->dispatch(frame->head.can_id,
                            frame->payload,
                            frame->head.dlc);
    }
    static void errHook(usb_rx_frame_t* /*frame*/) {}

    void dispatch(uint32_t id, const uint8_t* data, uint8_t len) {
        auto it = exact_map_.find(id);
        if (it != exact_map_.end()) { it->second(id, data, len); return; }
        for (auto& r : ranges_)
            if (id >= r.min && id <= r.max) { r.cb(id, data, len); return; }
        if (default_cb_) default_cb_(id, data, len);
    }
};

// 静态成员定义（放在某一个 .cpp）：
// DMTools* wheel_legged_hw::DMTools::instance_ = nullptr;

} // namespace wheel_legged_hw