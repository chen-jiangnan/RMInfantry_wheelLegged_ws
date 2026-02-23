# wheel_legged_sim

基于 MuJoCo 的轮腿机器人仿真包，通过 ROS2 topic 与控制层通信。

---

## 目录结构

```
wheel_legged_sim/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── wheel_legged_sim/
│       ├── mujoco_global.hpp         # 跨文件共享的全局变量声明（extern）
│       ├── simulation_interfaces.hpp # 线程安全的共享数据结构
│       ├── robot_bridge.hpp          # MuJoCo 数据读写桥接层
│       └── param.h                   # 运行时参数结构体
├── src/
│   ├── mujoco_sim.cpp                # MuJoCo 生命周期 + 物理循环 + main()
│   ├── simulation_brige_node.cpp     # ROS2 Node 类 + 工厂函数
│   ├── lodepng/                      # PNG 编解码库（MuJoCo 依赖）
│   │   ├── lodepng.cpp
│   │   ├── lodepng.h
│   │   └── ...
│   └── terrain_tool/                 # 地形生成工具（离线使用，不参与编译）
│       ├── terrain_generator.py      # 生成高度场 PNG
│       ├── scene.xml                 # 示例场景
│       └── readme_zh.md
└── mujoco -> /home/nan/.mujoco/mujoco-3.4.0/   # 符号链接，指向 MuJoCo 安装目录
  
```

### mujoco 符号链接

`mujoco/` 是一个指向系统 MuJoCo 安装目录的符号链接，CMakeLists 通过它引用：

- `mujoco/simulate/` — MuJoCo 官方 simulate 源文件（glfw_adapter、simulate.cc 等）
- `mujoco/include/` — MuJoCo 头文件
- `mujoco/lib/libmujoco.so` — 动态库

**创建符号链接：**

```bash
# 确认 MuJoCo 安装路径
ls ~/.mujoco/

# 在包目录下创建符号链接
cd ~/Desktop/RMInfantry_wheelLegged_ws/src/wheel_legged_sim
ln -s ~/.mujoco/mujoco-3.4.0 mujoco

# 验证
ls mujoco/include/mujoco/mujoco.h
```

如果 MuJoCo 安装在其他位置（如 `/opt/mujoco`），修改链接目标即可，CMakeLists 无需改动。

### lodepng

MuJoCo simulate 渲染截图功能依赖 lodepng，直接编译进可执行文件：

```bash
# 官方来源
# https://github.com/lvandeve/lodepng

# 只需要两个文件
src/lodepng/lodepng.cpp
src/lodepng/lodepng.h
```

CMakeLists 中已包含：

```cmake
file(GLOB SIM_SRC
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc
  src/lodepng/lodepng.cpp     # ← lodepng 在这里
)
```

### terrain_tool

离线地形生成工具，**不参与 ROS2 编译**，单独使用：

```bash
cd src/wheel_legged_sim/src/terrain_tool

# 安装依赖
pip install numpy pillow

# 生成高度场
python3 terrain_generator.py

# 输出：height_field.png（在 wheel_legged_description/cyclBot/terrain/ 下使用）
```

生成的 PNG 在 MJCF 中引用：

```xml
<asset>
  <hfield name="terrain" file="../../terrain/height_field.png"
          size="5 5 0.5 0.1"/>
</asset>
```

---

## 线程架构

```
┌─────────────────────────────────────────────────────────┐
│  main 线程          RenderLoop()         OpenGL 渲染  
│                                            
│  physics_thread     PhysicsThread()      
│    └─ PhysicsLoop()                    
│         └─ mj_step(m, d)          MuJoCo 物理步进   
│            bridge->run()          数据交换（同频）  
│                                  
│  ros_thread         rclcpp::spin()    ROS2 回调/定时器   
│                              
│  monitor_thread     监控 Ctrl+C，通知渲染退出          
└─────────────────────────────────────────────────────────┘
```

**关键设计：`bridge->run()` 紧跟在 `mj_step` 后面，在同一线程同频调用。**
不需要额外的数据交换线程，消除了原方案 `trylock` 可能丢数据的问题。

---

## 数据流

```
ROS 订阅回调线程
  └─ 写 g_sim_data.motor_cmd（std::atomic，无锁）

physics_thread（PhysicsLoop）
  └─ 读 motor_cmd → 写 d->ctrl
     mj_step(m, d)
     读 d->sensordata → 写 motor_state（std::atomic）
     读 d->sensordata → 写 IMU（std::mutex）

ROS timer 线程（500Hz）
  └─ 读 motor_state（std::atomic）→ 发布 JointStates
     读 IMU（std::mutex）→ 发布 IMUState
```

### 线程安全策略

| 数据类型      | 同步方式             | 原因                 |
| ------------- | -------------------- | -------------------- |
| 电机指令/状态 | `std::atomic`      | 单值读写，无锁高效   |
| IMU 数据      | `std::mutex`       | 多字段需要原子性读取 |
| 电机分组数量  | `std::atomic<int>` | 模型重载时动态更新   |

---

## 全局共享数据（mujoco_global.hpp）

```cpp
SimSharedData                g_sim_data;   // 所有线程共享的仿真数据
std::unique_ptr<RobotBridge> g_bridge;     // 物理线程持有，PhysicsLoop 调用 run()
std::atomic<bool>            g_bridge_ready; // bridge 初始化完成标志
```

`g_bridge_ready` 为 `false` 时，ROS timer 的 `publishAll()` 直接返回，
防止模型未加载或重载期间访问未初始化数据。

---

## 电机分组（动态识别）

电机分组数量从 MJCF 的 actuator 名称自动推断，支持不同配置的 XML：

| 场景      | chassis | gimbal | shoot |
| --------- | ------- | ------ | ----- |
| 纯底盘    | 6       | 0      | 0     |
| 底盘+云台 | 6       | 2      | 0     |
| 完整版    | 6       | 2      | 3     |

推断规则（`robot_sdk_bridge.hpp`）：

- 名称含 `gimbal` → 云台电机
- 名称含 `friction` /  `feeder` → 射击电机
- 其余 → 底盘电机

模型重载时（拖放或 UI 加载）自动重新推断，`publishAll()` 下次执行时自动适配。

---

## ROS2 接口

### 发布（仿真 → 控制）

| Topic                            | 消息类型        | 频率  | 内容                        |
| -------------------------------- | --------------- | ----- | --------------------------- |
| `simulation/chassisMotorState` | `JointStates` | 500Hz | 底盘 6 个电机 q/dq/tau      |
| `simulation/gimbalMotorState`  | `JointStates` | 500Hz | 云台电机状态（有则发）      |
| `simulation/shootMotorState`   | `JointStates` | 500Hz | 射击电机状态（有则发）      |
| `simulation/IMUState`          | `IMUState`    | 500Hz | 四元数/欧拉角/陀螺仪/加速度 |

### 订阅（控制 → 仿真）

| Topic                          | 消息类型      | 内容              |
| ------------------------------ | ------------- | ----------------- |
| `simulation/chassisMotorCmd` | `JointCmds` | 底盘电机 MIT 指令 |
| `simulation/gimbalMotorCmd`  | `JointCmds` | 云台电机指令      |
| `simulation/shootMotorCmd`   | `JointCmds` | 射击电机指令      |

### 电机控制模式（MIT 模式）

```
ctrl[i] = tau + kp * (q_target - q_actual) + kd * (dq_target - dq_actual)
```

### launch 文件 remapping

launch 文件将 topic 重映射到统一的 `lowlevel/` 命名空间，
与 `wheel_legged_hw` 实机节点保持一致，控制层无需区分仿真/实机：

```
simulation/chassisMotorState → lowlevel/chassisJointState
simulation/chassisMotorCmd   ← lowlevel/chassisJointCmd
simulation/IMUState          → lowlevel/IMUState
```

---

## MJCF 传感器约定

XML 文件中需要按以下命名定义传感器，`robot_sdk_bridge` 通过名称查找地址：

```xml
<!-- 电机传感器（顺序：position × N, velocity × N, torque × N）-->
<sensor>
  <jointpos  name="xxx_pos"    joint="xxx"/>
  <jointvel  name="xxx_vel"    joint="xxx"/>
  <actuatorfrc name="xxx_torque" actuator="xxx"/>
</sensor>

<!-- IMU 传感器 -->
<sensor>
  <framequat  name="chassis_imu_quat" objtype="site" objname="imu_site"/>
  <gyro       name="chassis_imu_gyro" site="imu_site"/>
  <accelerometer name="chassis_imu_acc" site="imu_site"/>
</sensor>

<!-- 底盘位置速度（可选）-->
<sensor>
  <framepos name="frame_pos" objtype="body" objname="base_link"/>
  <framelinvel name="frame_vel" objtype="body" objname="base_link"/>
</sensor>
```

---

## 参数说明

所有参数通过 ROS2 参数服务器管理，不再依赖 `config.yaml`。

| 参数                        | 类型   | 默认值             | 说明                                  |
| --------------------------- | ------ | ------------------ | ------------------------------------- |
| `robot`                   | string | `cyclBot`        | 机器人类型，对应 description 包子目录 |
| `robot_scene`             | string | `mjcf/scene.xml` | 相对于 description 包的场景路径       |
| `enable_elastic_band`     | bool   | `false`          | 是否启用弹性绳（辅助训练）            |
| `print_scene_information` | bool   | `false`          | 启动时打印 body/actuator/sensor 信息  |
| `band_attached_link`      | int    | `0`              | 弹性绳附着的 body id                  |

---

## 启动方式

```bash
# 方式一：launch 文件（推荐）
ros2 launch wheel_legged_sim mujoco_sim.launch.py

# 指定机器人和场景
ros2 launch wheel_legged_sim mujoco_sim.launch.py \
  robot:=cyclBot \
  robot_scene:=mjcf/scene_terrain.xml

# 方式二：直接运行并传参
ros2 run wheel_legged_sim mujoco_sim \
  --ros-args \
  -p robot:=cyclBot \
  -p robot_scene:=mjcf/scene.xml \
  -p print_scene_information:=true

# 打开场景信息调试输出
ros2 run wheel_legged_sim mujoco_sim \
  --ros-args -p print_scene_information:=true
```

---

## 弹性绳（Elastic Band）

用于辅助平衡训练，给机器人施加一个虚拟弹簧力防止倒地。

运行时键盘控制：

| 按键           | 功能                 |
| -------------- | -------------------- |
| `9`          | 开/关弹性绳          |
| `7` / `↑` | 缩短绳长（增大拉力） |
| `8` / `↓` | 延长绳长（减小拉力） |
| `Backspace`  | 重置仿真状态         |

参数（代码内调整）：

```cpp
elastic_band.stiffness_ = 200;   // 弹簧刚度 N/m
elastic_band.damping_   = 100;   // 阻尼系数 N·s/m
elastic_band.point_     = {0, 0, 3};  // 固定点坐标（世界系）
```

---

## 模型热重载

支持运行时更换模型，无需重启节点：

- **拖放**：将 `.xml` 文件拖入 MuJoCo 窗口
- **UI 加载**：MuJoCo 菜单 → Load

重载时 `g_bridge` 自动重建，电机分组重新推断，`g_bridge_ready` 保证重载期间不发布脏数据。

---

## 依赖

```
ROS2 Humble
MuJoCo 3.4.0
wheel_legged_msgs
wheel_legged_interfaces
wheel_legged_description    # 提供 MJCF/URDF 文件
ament_index_cpp
rclcpp
glfw3
fmt
```

---

## 编译

```bash
# Release（正常使用）
colcon build --packages-select wheel_legged_sim

# Debug（调试 segfault 等问题）
colcon build --packages-select wheel_legged_sim \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug

source install/setup.bash
```
