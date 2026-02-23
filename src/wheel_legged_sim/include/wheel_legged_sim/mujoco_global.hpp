#pragma once
#include <atomic>
#include <memory>
#include "simulation_interfaces.hpp"
#include "robot_bridge.hpp"

/* ================================================================
 *  两个文件共享的全局数据
 *  在 mujoco_sim.cpp 里定义，mujoco_sim_node.cpp 里 extern 引用
 * ================================================================ */
extern SimSharedData                g_sim_data;
extern std::unique_ptr<RobotBridge> g_bridge;
extern std::atomic<bool>            g_bridge_ready;