# wheel_legged_launch 使用说明

## 快速启动

### 仿真模式

```bash
# cyclBot 仿真（默认）
ros2 launch wheel_legged_launch bringup.launch.py

# seriBot 仿真
ros2 launch wheel_legged_launch bringup.launch.py robot_type:=seriBot env_mode:=sim
```

### 实机模式

```bash
# cyclBot 实机
ros2 launch wheel_legged_launch bringup.launch.py env_mode:=real

# seriBot 实机
ros2 launch wheel_legged_launch bringup.launch.py robot_type:=seriBot env_mode:=real
```

---

## 单独启动子模块

### 只启动仿真

```bash
ros2 launch wheel_legged_launch simulation.launch.py robot_type:=seriBot
```

### 只启动底盘控制

```bash
ros2 launch wheel_legged_launch chassis_control.launch.py robot_type:=seriBot
```

### 只启动可视化工具

```bash
ros2 launch wheel_legged_launch display.launch.py
```

---

## 启动时的节点列表

### env_mode:=sim

| 节点名               | 包                   | 说明        |
| -------------------- | -------------------- | ----------- |
| mujoco_sim_node      | wheel_legged_sim     | MuJoCo 仿真 |
| chassis_control_node | wheel_legged_control | 底盘控制    |
| gimbal_control_node  | wheel_legged_control | 云台控制    |
| shoot_control_node   | wheel_legged_control | 射击控制    |
| user_control_node    | wheel_legged_control | 用户指令    |
| joy_node             | joy                  | 手柄输入    |
| rviz2                | rviz2                | 可视化      |
| plotjuggler          | plotjuggler          | 数据曲线    |

### env_mode:=real（mujoco_sim_node 替换为 hw_node）

| 节点名  | 包              | 说明     |
| ------- | --------------- | -------- |
| hw_node | wheel_legged_hw | 硬件驱动 |
| ...     | 其余同上        |          |

---

## Topic 映射（仿真模式）

| 仿真内部 topic               | 对外 topic                 |
| ---------------------------- | -------------------------- |
| simulation/chassisMotorState | lowlevel/chassisJointState |
| simulation/chassisMotorCmd   | lowlevel/chassisJointCmd   |
| simulation/gimbalMotorState  | lowlevel/gimbalJointState  |
| simulation/gimbalMotorCmd    | lowlevel/gimbalJointCmd    |
| simulation/IMUState          | lowlevel/IMUState          |

---

## 配置文件路径

```
wheel_legged_launch/config/
├── cyclBot/
│   ├── chassis_control_config.yaml
│   ├── gimbal_control_config.yaml
│   ├── shoot_control_config.yaml
│   └── model/physical_params.yaml
└── seriBot/
    ├── chassis_control_config.yaml
    ├── gimbal_control_config.yaml
    ├── shoot_control_config.yaml
    └── model/physical_params.yaml
```

## 模型文件路径

```
wheel_legged_description/
├── cyclBot/mjcf/scene.xml   ← robot_type:=cyclBot 时加载
└── seriBot/mjcf/scene.xml   ← robot_type:=seriBot 时加载
```
