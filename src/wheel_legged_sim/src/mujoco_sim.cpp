// mujoco_sim.cpp
// 职责：MuJoCo 生命周期管理 + 物理循环
// 不包含任何 ROS2 发布/订阅逻辑

#define private public
#include "glfw_adapter.h"
#undef private

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>
#include <filesystem>

#include <mujoco/mujoco.h>
#include "simulate.h"
#include "array_safety.h"
#include "wheel_legged_sim/param.h"
#include "wheel_legged_sim/mujoco_global.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#include <sys/errno.h>
#include <unistd.h>
}

/* ================================================================
 *  全局共享数据定义（mujoco_sim_node.cpp 里 extern 引用）
 * ================================================================ */
SimSharedData                g_sim_data;
std::unique_ptr<RobotBridge> g_bridge;
std::atomic<bool>            g_bridge_ready{false};

namespace {
namespace mj  = ::mujoco;
namespace mju = ::mujoco::sample_util;

const double syncMisalign       = 0.1;
const double simRefreshFraction = 0.7;
const int    kErrorLength       = 1024;

mjModel* m         = nullptr;
mjData*  d         = nullptr;
mjtNum*  ctrlnoise = nullptr;

using Seconds = std::chrono::duration<double>;

/* ── 弹性绳 ── */
class ElasticBand {
public:
    void advance(std::vector<double> x, std::vector<double> dx) {
        std::vector<double> delta = {point_[0]-x[0], point_[1]-x[1], point_[2]-x[2]};
        double dist = sqrt(delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2]);
        if (dist < 1e-6) return;
        std::vector<double> dir = {delta[0]/dist, delta[1]/dist, delta[2]/dist};
        double v = dx[0]*dir[0] + dx[1]*dir[1] + dx[2]*dir[2];
        for (int i = 0; i < 3; ++i)
            f_[i] = (stiffness_*(dist-length_) - damping_*v) * dir[i];
    }
    double stiffness_          = 200;
    double damping_            = 100;
    std::vector<double> point_ = {0, 0, 3};
    double length_             = 0.0;
    bool   enable_             = false;
    std::vector<double> f_     = {0, 0, 0};
} elastic_band;

std::string getExecutableDir() {
    const char* path = "/proc/self/exe";
    std::unique_ptr<char[]> buf;
    uint32_t sz = 128;
    while (true) {
        buf.reset(new char[sz]);
        size_t n = readlink(path, buf.get(), sz);
        if (n < sz) { buf.get()[n] = '\0'; break; }
        sz *= 2;
    }
    std::string rp = buf.get();
    for (size_t i = rp.size()-1; i > 0; --i)
        if (rp[i] == '/') return rp.substr(0, i);
    return "";
}

void scanPluginLibraries() {
    int n = mjp_pluginCount();
    if (n) {
        printf("Built-in plugins:\n");
        for (int i = 0; i < n; ++i)
            printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
    std::string plugin_dir =
        std::filesystem::path(getExecutableDir()).parent_path().string()
        + "/" + MUJOCO_PLUGIN_DIR;
    mj_loadAllPluginLibraries(plugin_dir.c_str(),
        +[](const char* fn, int first, int cnt) {
            printf("Plugins from '%s':\n", fn);
            for (int i = first; i < first+cnt; ++i)
                printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        });
}

mjModel* LoadModel(const char* file, mj::Simulate& sim) {
    char filename[mj::Simulate::kMaxFilenameLength];
    mju::strcpy_arr(filename, file);
    if (!filename[0]) return nullptr;
    char err[kErrorLength] = "";
    mjModel* mnew = mj_loadXML(filename, nullptr, err, kErrorLength);
    if (err[0]) {
        int n = mju::strlen_arr(err);
        if (err[n-1]=='\n') err[n-1]='\0';
    }
    mju::strcpy_arr(sim.load_error, err);
    if (!mnew) { printf("%s\n", err); return nullptr; }
    if (err[0]) { printf("Model compiled with warning:\n  %s\n", err); sim.run=0; }
    return mnew;
}

/* 创建/重建 bridge 的辅助函数 */
void rebuildBridge() {
    g_bridge_ready.store(false);
    g_bridge = std::make_unique<RobotBridge>(m, d, g_sim_data);
    g_bridge_ready.store(true);
    printf("RobotBridge (re)initialized\n");
}

/* ================================================================
 *  PhysicsLoop
 *
 *  mj_step 之后立即调用 g_bridge->run()：
 *    写 d->ctrl ← g_sim_data.motor_cmd（来自 ROS 回调）
 *    读 d->sensordata → g_sim_data.motor_state（给 ROS timer）
 * ================================================================ */
void PhysicsLoop(mj::Simulate& sim) {
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    while (!sim.exitrequest.load() && rclcpp::ok()) {

        /* ── 处理模型重载请求 ── */
        auto reloadModel = [&](const char* fname) {
            sim.LoadMessage(fname);
            mjModel* mnew = LoadModel(fname, sim);
            mjData*  dnew = mnew ? mj_makeData(mnew) : nullptr;
            if (dnew) {
                sim.Load(mnew, dnew, fname);
                mj_deleteData(d); mj_deleteModel(m);
                m = mnew; d = dnew;
                mj_forward(m, d);
                free(ctrlnoise);
                ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
                mju_zero(ctrlnoise, m->nu);
                rebuildBridge();
            } else { sim.LoadMessageClear(); }
        };

        if (sim.droploadrequest.load()) {
            reloadModel(sim.dropfilename);
            sim.droploadrequest.store(false);
        }
        if (sim.uiloadrequest.load()) {
            sim.uiloadrequest.fetch_sub(1);
            reloadModel(sim.filename);
        }

        if (sim.run && sim.busywait) std::this_thread::yield();
        else std::this_thread::sleep_for(std::chrono::milliseconds(1));

        {
            const std::unique_lock<std::recursive_mutex> lock(sim.mtx);
            if (!m) continue;

            if (sim.run) {
                bool stepped = false;
                const auto startCPU   = mj::Simulate::Clock::now();
                const auto elapsedCPU = startCPU - syncCPU;
                double elapsedSim     = d->time - syncSim;
                double slowdown       = 100.0 / sim.percentRealTime[sim.real_time_index];
                bool misaligned = mju_abs(
                    Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

                auto applyBandAndStep = [&]() {
                    if (param::config.enable_elastic_band && elastic_band.enable_) {
                        elastic_band.advance(
                            {d->qpos[0], d->qpos[1], d->qpos[2]},
                            {d->qvel[0], d->qvel[1], d->qvel[2]});
                        int lnk = param::config.band_attached_link;
                        d->xfrc_applied[lnk]   = elastic_band.f_[0];
                        d->xfrc_applied[lnk+1] = elastic_band.f_[1];
                        d->xfrc_applied[lnk+2] = elastic_band.f_[2];
                    }
                    mj_step(m, d);                               // ← 物理步进
                    if (g_bridge_ready.load()) g_bridge->run();  // ← 数据交换
                    stepped = true;
                };

                if (elapsedSim < 0 || elapsedCPU.count() < 0 ||
                    syncCPU.time_since_epoch().count() == 0 ||
                    misaligned || sim.speed_changed)
                {
                    syncCPU = startCPU;
                    syncSim = d->time;
                    sim.speed_changed = false;
                    applyBandAndStep();
                } else {
                    bool measured  = false;
                    mjtNum prevSim = d->time;
                    double refreshTime = simRefreshFraction / sim.refresh_rate;
                    while (Seconds((d->time-syncSim)*slowdown) <
                           mj::Simulate::Clock::now()-syncCPU &&
                           mj::Simulate::Clock::now()-startCPU < Seconds(refreshTime))
                    {
                        if (!measured && elapsedSim) {
                            sim.measured_slowdown =
                                std::chrono::duration<double>(elapsedCPU).count()/elapsedSim;
                            measured = true;
                        }
                        applyBandAndStep();
                        if (d->time < prevSim) break;
                    }
                }
                if (stepped) sim.AddToHistory();
            } else {
                mj_forward(m, d);
                sim.speed_changed = true;
            }
        }
    }
}

} // namespace

/* ================================================================
 *  PhysicsThread（在 main 里 std::thread 调用）
 * ================================================================ */
void PhysicsThread(mj::Simulate* sim, const char* filename) {
    if (filename) {
        sim->LoadMessage(filename);
        m = LoadModel(filename, *sim);
        if (m) d = mj_makeData(m);
        if (d) {
            sim->Load(m, d, filename);
            mj_forward(m, d);
            free(ctrlnoise);
            ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
            mju_zero(ctrlnoise, m->nu);
            rebuildBridge();   // ← 模型加载完成后初始化 bridge
        } else { sim->LoadMessageClear(); }
    }
    PhysicsLoop(*sim);
    free(ctrlnoise);
}

/* ── 键盘回调 ── */
void user_key_cb(GLFWwindow*, int key, int, int act, int) {
    if (act != GLFW_PRESS) return;
    if (param::config.enable_elastic_band) {
        if      (key == GLFW_KEY_9)  elastic_band.enable_ = !elastic_band.enable_;
        else if (key == GLFW_KEY_7 || key == GLFW_KEY_UP)   elastic_band.length_ -= 0.1;
        else if (key == GLFW_KEY_8 || key == GLFW_KEY_DOWN) elastic_band.length_ += 0.1;
    }
    if (key == GLFW_KEY_BACKSPACE) { mj_resetData(m, d); mj_forward(m, d); }
}

std::string getMujocoExecutableDir() { return getExecutableDir(); }

/* ================================================================
 *  main —— 在 mujoco_sim.cpp 里，可以直接使用 MuJoCo/GLFW 类型
 * ================================================================ */

// MujocoSimNode 定义在 mujoco_sim_node.cpp，声明在这里
class MujocoSimNode;
// 用 shared_ptr 需要完整类型，改为在此 include node 头文件
// 实际上两个 cpp 编译到同一目标，链接时会找到定义
// 为避免重复 include，把 Node 的创建封装成工厂函数

// 前向声明 Node 工厂（定义在 mujoco_sim_node.cpp）
std::shared_ptr<rclcpp::Node> createMujocoSimNode(const std::string& name);
std::string getNodeRobot(std::shared_ptr<rclcpp::Node> node);
std::string getNodeRobotScene(std::shared_ptr<rclcpp::Node> node);

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    printf("MuJoCo version %s\n", mj_versionString());
    if (mjVERSION_HEADER != mj_version())
        mju_error("Headers and library have different versions");

    scanPluginLibraries();

    // Node 先创建，以便读取 ROS2 参数
    auto node = createMujocoSimNode("mujoco_sim_node");

    // 从 ROS2 参数构建场景路径
    // 使用 ament_index 查找 wheel_legged_description 包的 share 目录
    // 安装后路径：install/wheel_legged_description/share/wheel_legged_description/
    std::string robot       = getNodeRobot(node);
    std::string robot_scene = getNodeRobotScene(node);

    std::filesystem::path desc_share =
        ament_index_cpp::get_package_share_directory("wheel_legged_description");
    std::filesystem::path scene_path = desc_share / robot / "mjcf" /robot_scene;

    // MuJoCo 仿真对象
    mjvCamera cam;   mjv_defaultCamera(&cam);
    mjvOption opt;   mjv_defaultOption(&opt);
    mjvPerturb pert; mjv_defaultPerturb(&pert);

    auto sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, false);

    // 线程启动
    std::thread ros_thread([node] { rclcpp::spin(node); });
    std::thread physics_thread(&PhysicsThread, sim.get(), scene_path.c_str());
    std::thread monitor_thread([&sim] {
        while (rclcpp::ok() && !sim->exitrequest.load())
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!rclcpp::ok()) sim->exitrequest.store(true);
    });

    printf("Starting RenderLoop...\n");
    sim->RenderLoop();
    printf("RenderLoop exited\n");  // 如果没打印说明在 RenderLoop 里崩的

    // 退出清理
    sim->exitrequest.store(true);
    if (rclcpp::ok()) rclcpp::shutdown();

    if (monitor_thread.joinable())  monitor_thread.join();
    if (physics_thread.joinable())  physics_thread.join();
    if (ros_thread.joinable())      ros_thread.join();

    g_bridge.reset();
    if (d) { mj_deleteData(d); d = nullptr; }
    if (m) { mj_deleteModel(m); m = nullptr; }

    return 0;
}