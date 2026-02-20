# 达妙 USB-CANFD SDK 使用说明

------

## 1. 简介

本 SDK 提供了对达妙USB-CANFD 系列设备的访问接口，支持如下设备类型：

- `DEV_USB2CANFD`：单通道 USB-CANFD 设备
- `DEV_USB2CANFD_DUAL`：双通道 USB-CANFD 设备
- `DEV_ECAT2CANFD`：EtherCAT 转 CANFD 设备

SDK 支持 Windows 和 Linux 平台，提供 C 接口，可用于 C/C++ 项目中。

------

## 2. 快速开始

### 2.1 编译依赖

- **Windows**：使用 Visual Studio，需链接 `.lib` 导入库并包含头文件 `pub_user.h`
- **Linux**：使用 GCC/G++，需链接动态库（如 `libdevice.so`）并包含头文件

确保将 SDK 的头文件路径和库路径添加到你的构建系统中。

------

### 2.2 基本使用流程

以下为典型使用步骤（参考 `main.cpp`）：

```c++
#include "pub_user.h"
```

#### (1) 创建设备句柄

```c++
damiao_handle* handle = damiao_handle_create(DEV_USB2CANFD_DUAL);
```

#### (2) 查找并打开设备

```c++
int device_cnt = damiao_handle_find_devices(handle);
if (device_cnt == 0) { /* 无设备 */ }

device_handle* dev_list[16];
int handle_cnt = 0;
damiao_handle_get_devices(handle, dev_list, &handle_cnt);

if (!device_open(dev_list[0])) { /* 打开失败 */ }
```

#### (3) 获取设备信息（可选）

```c++
char buf[255];
device_get_version(dev_list[0], buf, sizeof(buf));
device_get_serial_number(dev_list[0], buf, sizeof(buf));
```

#### (4) 配置 CAN 通道波特率

```c++
// 设置通道 0 为 CAN FD 模式，仲裁波特率 1Mbps，数据波特率 5Mbps，采样点 75%
device_channel_set_baud_with_sp(dev_list[0], 0, true, 1000000, 5000000, 0.75f, 0.75f);
device_open_channel(dev_list[0], 0);
```

> 同样可配置通道 1（双通道设备）

#### (5) 注册回调函数

```c++
void rec_callback(usb_rx_frame_t* frame) 
{
    printf("Received ID: 0x%x\n", frame->head.can_id);
}

void sent_callback(usb_rx_frame_t* frame) 
{
    printf("Sent ID: 0x%x\n", frame->head.can_id);
}

device_hook_to_rec(dev_list[0], rec_callback);
device_hook_to_sent(dev_list[0], sent_callback);
```

#### (6) 发送 CAN/CANFD 帧

**快速发送（推荐用于简单场景）：**

```c++
uint8_t payload[8] = {1,2,3,4,5,6,7,8};
device_channel_send_fast(
    dev_list[0],   // 设备句柄
    0,             // 通道号
    0x123,         // CAN ID
    1,             // 发送次数（-1 表示持续发送）
    false,         // 是否扩展帧
    true,          // 是否 CAN FD
    true,          // 是否启用 BRS（波特率切换）
    8,             // 数据长度
    payload        // 数据内容
);
```

**高级发送（支持 ID 自增、步进控制等）：**

使用 `device_channel_send_advanced()`，详情见头文件定义。

> ⚠️ 注意：发送前必须确保设备已连接到有效的 CAN 总线，否则可能无响应。

#### (7) 清理资源

```c++
device_close(dev_list[0]);
damiao_handle_destroy(handle);
```

------

## 3. 关键数据结构说明

### 3.1 `usb_tx_frame_t`（发送帧）

- `head`：包含 CAN ID、帧类型（标准/扩展、数据/远程）、CANFD/BRS、通道、发送次数等
- `payload[64]`：最多 64 字节数据（CANFD 支持）

### 3.2 `usb_rx_frame_t`（接收帧）

- `head`：包含时间戳、方向（rx/tx）、ACK、通道等
- `payload[64]`：接收到的数据

### 3.3 波特率配置

使用 `device_baud_t` 或直接调用 `device_channel_set_baud_with_sp()` 设置：

- `bitrate`：仲裁段波特率（如 1 Mbps）
- `dbitrate`：数据段波特率（仅 CAN FD 有效，如 5 Mbps）
- `can_sp` / `canfd_sp`：采样点（0.0 ~ 1.0，建议 0.75~0.80）

------

## 4. 回调机制

SDK 采用异步回调方式处理收发事件：

| 回调类型 | 函数指针            | 触发时机                       |
| -------- | ------------------- | ------------------------------ |
| 接收回调 | `dev_rec_callback`  | 收到 CAN 帧时                  |
| 发送回显 | `dev_sent_callback` | 成功发送后（回显）             |
| 错误回调 | `dev_err_callback`  | 发生错误时（暂未在示例中使用） |

> 所有回调均在 SDK 内部线程中执行，请注意线程安全。

------

## 5. 注意事项

- **通道编号**：从 `0` 开始（双通道设备为 0 和 1）
- **CAN ID 范围**：
  - 标准帧：0x000 ~ 0x7FF
  - 扩展帧：0x00000000 ~ 0x1FFFFFFF
- **数据长度**：
  - CAN 2.0：最大 8 字节
  - CAN FD：最大 64 字节（需硬件支持）
- **发送间隔与次数**：在 `usb_tx_frame_head_t` 中可通过 `interval` 和 `send_times` 控制（高级发送接口）
- **务必调用 `device_close()` 和 `damiao_handle_destroy()` 释放资源**

------

## 6. 示例项目

`main.cpp` 提供了一个完整的双通道初始化、配置、回调注册示例。默认注释掉了发送循环，取消注释即可测试发送功能：

```c++
// 取消以下注释以发送测试帧（需连接 CAN 总线！）
// device_channel_send_fast(...);
// std::this_thread::sleep_for(...);
```

------

## 7. 技术支持

如遇问题，请提供：

- 设备型号与序列号
- 操作系统版本
- SDK 版本（可通过 `damiao_print_version()` 获取）
- 错误现象与日志

------

## 8. Bug 反馈与 Issue 提交

如果您在使用本 SDK 过程中遇到问题、发现 Bug，或有功能改进建议，欢迎通过 Gitee/Github issues 向我们反馈！

### 提交 Issue 前请准备以下信息：

- **SDK 版本号**（可通过 `damiao_print_version()` 获取）
- **设备型号与序列号**
- **操作系统及版本**（如 Windows 11 23H2 / Ubuntu 22.04 LTS）
- **编译环境**（如 Visual Studio 2022 / GCC 11.4）
- **问题复现步骤**（越详细越好）
- **错误日志或截图**（如有）
- **最小可复现代码示例**（强烈建议提供）

> 请勿在 Issue 中泄露敏感信息（如公司内部代码、密钥等）。

我们会在收到反馈后尽快响应。感谢您的支持与贡献！

> © 2025 Damiao Technology. All rights reserved.