// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// !!! hack code: make glfw_adapter.window_ public
#include <cmath>
#include <cstddef>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <wheel_legged_msgs/msg/detail/imu_state__struct.hpp>
#define private public
#include "glfw_adapter.h"
#undef private

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include "simulate.h"
#include "array_safety.h"
#include "wheel_legged_sim/param.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "wheel_legged_sim/robot_sdk_bridge.hpp"
#include "wheel_legged_sim/simulation_interfaces.hpp"

#include "rclcpp/rclcpp.hpp"

#include <thread>
#include <atomic>

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C"
{
#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#else
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#include <sys/errno.h>
#include <unistd.h>
#endif
}

class ElasticBand
{
public:
  ElasticBand(){};
  void Advance(std::vector<double> x, std::vector<double> dx)
  {
    std::vector<double> delta_x = {0.0, 0.0, 0.0};
    delta_x[0] = point_[0] - x[0];
    delta_x[1] = point_[1] - x[1];
    delta_x[2] = point_[2] - x[2];
    double distance = sqrt(delta_x[0] * delta_x[0] + delta_x[1] * delta_x[1] + delta_x[2] * delta_x[2]);

    std::vector<double> direction = {0.0, 0.0, 0.0};
    direction[0] = delta_x[0] / distance;
    direction[1] = delta_x[1] / distance;
    direction[2] = delta_x[2] / distance;

    double v = dx[0] * direction[0] + dx[1] * direction[1] + dx[2] * direction[2];

    f_[0] = (stiffness_ * (distance - length_) - damping_ * v) * direction[0];
    f_[1] = (stiffness_ * (distance - length_) - damping_ * v) * direction[1];
    f_[2] = (stiffness_ * (distance - length_) - damping_ * v) * direction[2];
  }


  double stiffness_ = 200;
  double damping_ = 100;
  std::vector<double> point_ = {0, 0, 3};
  double length_ = 0.0;
  bool enable_ = true;
  std::vector<double> f_ = {0, 0, 0};
};
inline ElasticBand elastic_band;


namespace
{
  namespace mj = ::mujoco;
  namespace mju = ::mujoco::sample_util;

  // constants
  const double syncMisalign = 0.1;       // maximum mis-alignment before re-sync (simulation seconds)
  const double simRefreshFraction = 0.7; // fraction of refresh available for simulation
  const int kErrorLength = 1024;         // load error string length

  // model and data
  mjModel *m = nullptr;
  mjData *d = nullptr;

  // control noise variables
  mjtNum *ctrlnoise = nullptr;

  using Seconds = std::chrono::duration<double>;

  //---------------------------------------- plugin handling -----------------------------------------

  // return the path to the directory containing the current executable
  // used to determine the location of auto-loaded plugin libraries
  std::string getExecutableDir()
  {
#if defined(_WIN32) || defined(__CYGWIN__)
    constexpr char kPathSep = '\\';
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      DWORD buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
        if (written < buf_size)
        {
          success = true;
        }
        else if (written == buf_size)
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
        else
        {
          std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
          return "";
        }
      }
      return realpath.get();
    }();
#else
    constexpr char kPathSep = '/';
#if defined(__APPLE__)
    std::unique_ptr<char[]> buf(nullptr);
    {
      std::uint32_t buf_size = 0;
      _NSGetExecutablePath(nullptr, &buf_size);
      buf.reset(new char[buf_size]);
      if (!buf)
      {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }
      if (_NSGetExecutablePath(buf.get(), &buf_size))
      {
        std::cerr << "unexpected error from _NSGetExecutablePath\n";
      }
    }
    const char *path = buf.get();
#else
    const char *path = "/proc/self/exe";
#endif
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      std::uint32_t buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        std::size_t written = readlink(path, realpath.get(), buf_size);
        if (written < buf_size)
        {
          realpath.get()[written] = '\0';
          success = true;
        }
        else if (written == -1)
        {
          if (errno == EINVAL)
          {
            // path is already not a symlink, just use it
            return path;
          }

          std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
          return "";
        }
        else
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
      }
      return realpath.get();
    }();
#endif

    if (realpath.empty())
    {
      return "";
    }

    for (std::size_t i = realpath.size() - 1; i > 0; --i)
    {
      if (realpath.c_str()[i] == kPathSep)
      {
        return realpath.substr(0, i);
      }
    }

    // don't scan through the entire file system's root
    return "";
  }

  // scan for libraries in the plugin directory to load additional plugins
  void scanPluginLibraries()
  {
    // check and print plugins that are linked directly into the executable
    int nplugin = mjp_pluginCount();
    if (nplugin)
    {
      std::printf("Built-in plugins:\n");
      for (int i = 0; i < nplugin; ++i)
      {
        std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
      }
    }

    // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
    const std::string sep = "\\";
#else
    const std::string sep = "/";
#endif

    // try to open the ${EXECDIR}/plugin directory
    // ${EXECDIR} is the directory containing the simulate binary itself
    const std::string executable_dir = getExecutableDir();
    if (executable_dir.empty())
    {
      return;
    }
    ;
    const std::string plugin_dir = std::filesystem::path(getExecutableDir()).parent_path().c_str() + sep + MUJOCO_PLUGIN_DIR;
    mj_loadAllPluginLibraries(
        plugin_dir.c_str(), +[](const char *filename, int first, int count)
                            {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        } });
  }

  //------------------------------------------- simulation -------------------------------------------

  mjModel *LoadModel(const char *file, mj::Simulate &sim)
  {
    // this copy is needed so that the mju::strlen call below compiles
    char filename[mj::Simulate::kMaxFilenameLength];
    mju::strcpy_arr(filename, file);

    // make sure filename is not empty
    if (!filename[0])
    {
      return nullptr;
    }

    // load and compile
    char loadError[kErrorLength] = "";
    mjModel *mnew = 0;
    if (mju::strlen_arr(filename) > 4 &&
        !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                      mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
    {
      mnew = mj_loadModel(filename, nullptr);
      if (!mnew)
      {
        mju::strcpy_arr(loadError, "could not load binary model");
      }
    }
    else
    {
      mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
      // remove trailing newline character from loadError
      if (loadError[0])
      {
        int error_length = mju::strlen_arr(loadError);
        if (loadError[error_length - 1] == '\n')
        {
          loadError[error_length - 1] = '\0';
        }
      }
    }

    mju::strcpy_arr(sim.load_error, loadError);

    if (!mnew)
    {
      std::printf("%s\n", loadError);
      return nullptr;
    }

    // compiler warning: print and pause
    if (loadError[0])
    {
      // mj_forward() below will print the warning message
      std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
      sim.run = 0;
    }

    return mnew;
  }

  // simulate in background thread (while rendering in main thread)
  void PhysicsLoop(mj::Simulate &sim)
  {
    // cpu-sim syncronization point
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    // run until asked to exit
    while (!sim.exitrequest.load() && rclcpp::ok())
    {
      if (sim.droploadrequest.load())
      {
        sim.LoadMessage(sim.dropfilename);
        mjModel *mnew = LoadModel(sim.dropfilename, sim);
        sim.droploadrequest.store(false);

        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.dropfilename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          mj_forward(m, d);

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * m->nu);
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      if (sim.uiloadrequest.load())
      {
        sim.uiloadrequest.fetch_sub(1);
        sim.LoadMessage(sim.filename);
        mjModel *mnew = LoadModel(sim.filename, sim);
        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.filename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          mj_forward(m, d);

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      // sleep for 1 ms or yield, to let main thread run
      //  yield results in busy wait - which has better timing but kills battery life
      if (sim.run && sim.busywait)
      {
        std::this_thread::yield();
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      {
        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        // run only if model is present
        if (m)
        {
          // running
          if (sim.run)
          {
            bool stepped = false;

            // record cpu time at start of iteration
            const auto startCPU = mj::Simulate::Clock::now();

            // elapsed CPU and simulation time since last sync
            const auto elapsedCPU = startCPU - syncCPU;
            double elapsedSim = d->time - syncSim;

            // inject noise
            if (sim.ctrl_noise_std)
            {
              // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
              mjtNum rate = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
              mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1 - rate * rate);

              for (int i = 0; i < m->nu; i++)
              {
                // update noise
                ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

                // apply noise
                d->ctrl[i] = ctrlnoise[i];
              }
            }

            // requested slow-down factor
            double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

            // misalignment condition: distance from target sim time is bigger than syncmisalign
            bool misaligned =
                mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

            // out-of-sync (for any reason): reset sync times, step
            if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
                misaligned || sim.speed_changed)
            {
              // re-sync
              syncCPU = startCPU;
              syncSim = d->time;
              sim.speed_changed = false;

              // run single step, let next iteration deal with timing
              mj_step(m, d);
              stepped = true;
            }

            // in-sync: step until ahead of cpu
            else
            {
              bool measured = false;
              mjtNum prevSim = d->time;

              double refreshTime = simRefreshFraction / sim.refresh_rate;

              // step while sim lags behind cpu and within refreshTime
              while (Seconds((d->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                     mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime))
              {
                // measure slowdown before first step
                if (!measured && elapsedSim)
                {
                  sim.measured_slowdown =
                      std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                  measured = true;
                }

                // elastic band on base link
                if (param::config.enable_elastic_band == 1)
                {
                  if (elastic_band.enable_)
                  {
                    std::vector<double> x = {d->qpos[0], d->qpos[1], d->qpos[2]};
                    std::vector<double> dx = {d->qvel[0], d->qvel[1], d->qvel[2]};

                    elastic_band.Advance(x, dx);

                    d->xfrc_applied[param::config.band_attached_link] = elastic_band.f_[0];
                    d->xfrc_applied[param::config.band_attached_link + 1] = elastic_band.f_[1];
                    d->xfrc_applied[param::config.band_attached_link + 2] = elastic_band.f_[2];
                  }
                }

                // call mj_step
                mj_step(m, d);
                stepped = true;

                // break if reset
                if (d->time < prevSim)
                {
                  break;
                }
              }
            }

            // save current state to history buffer
            if (stepped)
            {
              sim.AddToHistory();
            }
          }

          // paused
          else
          {
            // run mj_forward, to update rendering and joint sliders
            mj_forward(m, d);
            sim.speed_changed = true;
          }
        }
      } // release std::lock_guard<std::mutex>
    }
  }
  
  // 提供访问 m 和 d 的函数
  mjModel* getMjModel() { return m; }
  mjData* getMjData() { return d; }
} // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate *sim, const char *filename)
{
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr)
  {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m)
      d = mj_makeData(m);
    if (d)
    {
      sim->Load(m, d, filename);
      mj_forward(m, d);

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
      mju_zero(ctrlnoise, m->nu);
    }
    else
    {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  free(ctrlnoise);
  // mj_deleteData(d);
  // mj_deleteModel(m);
  // exit(0);
}


//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char *title, const char *msg);
static const char *rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char *msg)
{
  rosetta_error_msg = msg;
}
#endif

// user keyboard callback
void user_key_cb(GLFWwindow* window, int key, int scancode, int act, int mods) {
  if (act==GLFW_PRESS)
  {
    if(param::config.enable_elastic_band == 1) {
      if (key==GLFW_KEY_9) {
        elastic_band.enable_ = !elastic_band.enable_;
      } else if (key==GLFW_KEY_7 || key==GLFW_KEY_UP) {
        elastic_band.length_ -= 0.1;
      } else if (key==GLFW_KEY_8 || key==GLFW_KEY_DOWN) {
        elastic_band.length_ += 0.1;
      }
    }
    if(key==GLFW_KEY_BACKSPACE) {
      mj_resetData(m, d);
      mj_forward(m, d);
    }
  }
}

//=====================================================ROS2 NODE===========================================
class MujocoSimNode : public rclcpp::Node {
public:
  MujocoSimNode(std::string name) :Node(name){
    RCLCPP_INFO(this->get_logger(), "%s节点已启动.", name.c_str());

    // 创建 chassisMotorState 发布者
    chassis_motorState_publisher_ = this->create_publisher<wheel_legged_msgs::msg::ChassisJointState>(
      "simulation/chassisMotorState", 10);
    // 创建 chassisMotorCmd 订阅者  
    chassis_motorCmd_subscriber_ = this->create_subscription<wheel_legged_msgs::msg::ChassisJointCmd>(
      "simulation/chassisMotorCmd", 10, 
      std::bind(&MujocoSimNode::ChassisMotorCmd_callback, this, std::placeholders::_1));

    // 创建 GimablMotorState 发布者
    gimbal_motorState_publisher_ = this->create_publisher<wheel_legged_msgs::msg::GimbalJointState>(
      "simulation/gimbalMotorState", 10);
    // 创建 shootMotorCmd 订阅者  
    gimbal_motorCmd_subscriber_ = this->create_subscription<wheel_legged_msgs::msg::GimbalJointCmd>(
      "simulation/gimbalMotorCmd", 10, 
      std::bind(&MujocoSimNode::GimbalMotorCmd_callback, this, std::placeholders::_1));

    // 创建 shootMotorState 发布者
    shoot_motorState_publisher_ = this->create_publisher<wheel_legged_msgs::msg::ShootJointState>(
      "simulation/shootMotorState", 10);
    // 创建 shootMotorCmd 订阅者  
    shoot_motorCmd_subscriber_ = this->create_subscription<wheel_legged_msgs::msg::ShootJointCmd>(
      "simulation/shootMotorCmd", 10, 
      std::bind(&MujocoSimNode::ShootMotorCmd_callback, this, std::placeholders::_1));

    // 创建定时器发布 all motor state
    all_motorState_pubTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),  // 1000Hz
      std::bind(&MujocoSimNode::AllMotorState_Publish, this));

    // 创建 shootMotorState 发布者
    chassis_imu_publisher_ = this->create_publisher<wheel_legged_msgs::msg::IMUState>(
      "simulation/IMUState", 10);
    // 创建定时器发布 imu data
    chassis_imu_pubTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),  // 1000Hz
      std::bind(&MujocoSimNode::ChassisIMU_Publish, this));

    // 创建数据交换线程
    data_exchange_running_ = false;
    data_exchange_thread_ = std::thread(&MujocoSimNode::DataExchangeLoop, this);
  }
  ~MujocoSimNode() {
    data_exchange_running_ = false;
    if (data_exchange_thread_.joinable()) {
      data_exchange_thread_.join();
    }
  }
private:
  // 数据交换线程：从 mjdata 读取到 lowstate，从 lowcmd 写入到 mjdata
  void DataExchangeLoop() {
    // 等待mjdata初始化完成
    while(rclcpp::ok()){
      if(m != nullptr && d != nullptr){
        std::cout << "Mujoco data is prepared" << std::endl;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    data_exchange_running_ = true;
    // 创建 robot_bridge
    robot_bridge_ = std::make_shared<RobotBridge>(m, d);
    while (rclcpp::ok() ) {

      robot_bridge_->run();

      std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1kHz
    }
  }

  // lowcmd 回调函数
  void ChassisMotorCmd_callback(const wheel_legged_msgs::msg::ChassisJointCmd::SharedPtr msg) {
    if(data_exchange_running_){
      // std::lock_guard<std::mutex> lock(robot_bridge_->lowcmd->mutex_);
      for(size_t i = 0; i < 6; i++) {
        if(i < msg->joint_cmd.size()) {
          auto & motor_cmd = robot_bridge_->lowcmd->motor_state()[i];
          motor_cmd.mode() = msg->joint_cmd[i].mode;
          motor_cmd.q() = static_cast<double>(msg->joint_cmd[i].q);
          motor_cmd.dq() = static_cast<double>(msg->joint_cmd[i].dq);
          motor_cmd.tau() = static_cast<double>(msg->joint_cmd[i].tau);
          motor_cmd.p_kp() = static_cast<double>(msg->joint_cmd[i].p_kp);
          motor_cmd.p_kd() = static_cast<double>(msg->joint_cmd[i].p_kd);
          motor_cmd.v_kp() = static_cast<double>(msg->joint_cmd[i].v_kp);
          motor_cmd.v_kd() = static_cast<double>(msg->joint_cmd[i].v_kd);
        }
      }
    }
  }
  void GimbalMotorCmd_callback(const wheel_legged_msgs::msg::GimbalJointCmd::SharedPtr msg) {
    if(data_exchange_running_){
      // std::lock_guard<std::mutex> lock(robot_bridge_->lowcmd->mutex_);
      for(size_t i = 0; i < 3; i++) {
        if(i < msg->joint_cmd.size()) {
          auto & motor_cmd = robot_bridge_->lowcmd->motor_state()[i+6];
          motor_cmd.mode() = msg->joint_cmd[i+6].mode;
          motor_cmd.q() = static_cast<double>(msg->joint_cmd[i+6].q);
          motor_cmd.dq() = static_cast<double>(msg->joint_cmd[i+6].dq);
          motor_cmd.tau() = static_cast<double>(msg->joint_cmd[i+6].tau);
          motor_cmd.p_kp() = static_cast<double>(msg->joint_cmd[i+6].p_kp);
          motor_cmd.p_kd() = static_cast<double>(msg->joint_cmd[i+6].p_kd);
          motor_cmd.v_kp() = static_cast<double>(msg->joint_cmd[i+6].v_kp);
          motor_cmd.v_kd() = static_cast<double>(msg->joint_cmd[i+6].v_kd);
        }
      }
    }
  }
  void ShootMotorCmd_callback(const wheel_legged_msgs::msg::ShootJointCmd::SharedPtr msg) {
    if(data_exchange_running_){
      // std::lock_guard<std::mutex> lock(robot_bridge_->lowcmd->mutex_);
      for(size_t i = 0; i < 3; i++) {
        if(i < msg->joint_cmd.size()) {
          auto & motor_cmd = robot_bridge_->lowcmd->motor_state()[i+8];
          motor_cmd.mode() = msg->joint_cmd[i+8].mode;
          motor_cmd.q() = static_cast<double>(msg->joint_cmd[i+8].q);
          motor_cmd.dq() = static_cast<double>(msg->joint_cmd[i+8].dq);
          motor_cmd.tau() = static_cast<double>(msg->joint_cmd[i+8].tau);
          motor_cmd.p_kp() = static_cast<double>(msg->joint_cmd[i+8].p_kp);
          motor_cmd.p_kd() = static_cast<double>(msg->joint_cmd[i+8].p_kd);
          motor_cmd.v_kp() = static_cast<double>(msg->joint_cmd[i+8].v_kp);
          motor_cmd.v_kd() = static_cast<double>(msg->joint_cmd[i+8].v_kd);
        }
      }
    }
  }
  
  // 发布 lowstate
  void AllMotorState_Publish() {
    if(data_exchange_running_) {
      // 更新 chassis 电机状态
      auto chassis_msg = wheel_legged_msgs::msg::ChassisJointState();
      for(size_t i = 0; i < chassis_msg.joint_state.size(); i++) {
        chassis_msg.joint_state[i].mode = robot_bridge_->lowstate->motor_state()[i].mode();
        chassis_msg.joint_state[i].q = static_cast<float>(robot_bridge_->lowstate->motor_state()[i].q());
        chassis_msg.joint_state[i].dq = static_cast<float>(robot_bridge_->lowstate->motor_state()[i].dq());
        chassis_msg.joint_state[i].tau = static_cast<float>(robot_bridge_->lowstate->motor_state()[i].tau_est());
        chassis_msg.joint_state[i].tau_est = static_cast<float>(robot_bridge_->lowstate->motor_state()[i].tau_est());
      }chassis_motorState_publisher_->publish(chassis_msg);

      // 更新 gimbal 电机状态
      auto gimbal_msg = wheel_legged_msgs::msg::GimbalJointState();
      for(size_t i = 0, j = chassis_msg.joint_state.size(); i < gimbal_msg.joint_state.size(); i++) {
        gimbal_msg.joint_state[i].mode = robot_bridge_->lowstate->motor_state()[i+j].mode();
        gimbal_msg.joint_state[i].q = static_cast<float>(robot_bridge_->lowstate->motor_state()[i+j].q());
        gimbal_msg.joint_state[i].dq = static_cast<float>(robot_bridge_->lowstate->motor_state()[i+j].dq());
        gimbal_msg.joint_state[i].tau = static_cast<float>(robot_bridge_->lowstate->motor_state()[i+j].tau_est());
        gimbal_msg.joint_state[i].tau_est = static_cast<float>(robot_bridge_->lowstate->motor_state()[i+j].tau_est());
      }gimbal_motorState_publisher_->publish(gimbal_msg);
      
      // 更新 shoot 电机状态
      auto shoot_msg = wheel_legged_msgs::msg::ShootJointState();
      for(size_t i = 0, j=chassis_msg.joint_state.size()+gimbal_msg.joint_state.size(); i < shoot_msg.joint_state.size(); i++) {
        shoot_msg.joint_state[i].mode = robot_bridge_->lowstate->motor_state()[i+j].mode();
        shoot_msg.joint_state[i].q = static_cast<float>(robot_bridge_->lowstate->motor_state()[i+j].q());
        shoot_msg.joint_state[i].dq = static_cast<float>(robot_bridge_->lowstate->motor_state()[i+j].dq());
        shoot_msg.joint_state[i].tau = static_cast<float>(robot_bridge_->lowstate->motor_state()[i+j].tau_est());
        shoot_msg.joint_state[i].tau_est = static_cast<float>(robot_bridge_->lowstate->motor_state()[i+j].tau_est());
      }shoot_motorState_publisher_->publish(shoot_msg);   
    }
  }

  void ChassisIMU_Publish(){
    if(data_exchange_running_) {
      // publish chassis imu data
      auto chassis_imu_msg = wheel_legged_msgs::msg::IMUState();
      chassis_imu_msg.header.stamp = this->now();
      chassis_imu_msg.header.frame_id = "chassis_imu";
      for(int i = 0; i < 4; i++){
        chassis_imu_msg.quaternion[i] = robot_bridge_->lowstate->chassis_imu_state()->quaternion()[i];
      }
      for(int i = 0; i < 3; i++){
        chassis_imu_msg.gyroscope[i] = robot_bridge_->lowstate->chassis_imu_state()->gyroscope()[i];
        chassis_imu_msg.accelerometer[i] = robot_bridge_->lowstate->chassis_imu_state()->accelerometer()[i];
        chassis_imu_msg.rpy[i] = robot_bridge_->lowstate->chassis_imu_state()->rpy()[i];
      }
      chassis_imu_publisher_->publish(chassis_imu_msg);
    }
  }
  std::shared_ptr<RobotBridge> robot_bridge_;
  std::thread data_exchange_thread_;
  std::atomic<bool> data_exchange_running_;

  rclcpp::Publisher<wheel_legged_msgs::msg::ChassisJointState>::SharedPtr chassis_motorState_publisher_;
  rclcpp::Subscription<wheel_legged_msgs::msg::ChassisJointCmd>::SharedPtr chassis_motorCmd_subscriber_;
  rclcpp::Publisher<wheel_legged_msgs::msg::GimbalJointState>::SharedPtr gimbal_motorState_publisher_;
  rclcpp::Subscription<wheel_legged_msgs::msg::GimbalJointCmd>::SharedPtr gimbal_motorCmd_subscriber_;
  rclcpp::Publisher<wheel_legged_msgs::msg::ShootJointState>::SharedPtr shoot_motorState_publisher_;
  rclcpp::Subscription<wheel_legged_msgs::msg::ShootJointCmd>::SharedPtr shoot_motorCmd_subscriber_;
  rclcpp::TimerBase::SharedPtr all_motorState_pubTimer_;

  rclcpp::Publisher<wheel_legged_msgs::msg::IMUState>::SharedPtr chassis_imu_publisher_;
  // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gimbal_imu_publisher_;
  rclcpp::TimerBase::SharedPtr chassis_imu_pubTimer_;
};

// run event loop
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg)
  {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version())
  {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // Load simulation configuration
  std::filesystem::path proj_dir = std::filesystem::path(getExecutableDir()).parent_path().parent_path(); 
  param::config.load_from_yaml(proj_dir / "share"/ PACKAGE_NAME / "config.yaml");
  param::helper(argc, argv);
  if(param::config.robot_scene.is_relative()) {
    param::config.robot_scene = proj_dir.parent_path().parent_path()/ "src" / "wheel_legged_description" / param::config.robot_file_path;
  }

  // simulate object encapsulates the UIb
  auto sim = std::make_unique<mj::Simulate>(
    std::make_unique<mj::GlfwAdapter>(),
    &cam, &opt, &pert, /* is_passive = */ false);

  // start ros2 spin thread
  auto node = std::make_shared<MujocoSimNode>("mujoco_sim_node");
  std::thread ros2_spin_thread([node]() {rclcpp::spin(node);});
  
  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), param::config.robot_scene.c_str());

  // 如果检测到 Ctrl+C (ROS shutdown)，强制让 RenderLoop 退出
  std::thread monitor_thread([&sim]() {
    while (rclcpp::ok() && !sim->exitrequest.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!rclcpp::ok()) {
        sim->exitrequest.store(true); // 强制 UI 退出
    }
  });
  // start simulation UI loop (blocking call)
  sim->RenderLoop();

  // ================= 退出清理阶段 =================

  // A. 首先通知物理线程停止 (如果是手动关窗口触发的这里)
  sim->exitrequest.store(true);

  // B. 显式关闭 ROS，这会让 ros2_spin_thread 和 monitor_thread 结束循环
  if (rclcpp::ok()) {
      rclcpp::shutdown();
  }
  // 回收监控线程
  if (monitor_thread.joinable()) {
      monitor_thread.join();
  }
  // 回收物理线程
  if (physicsthreadhandle.joinable()) {
      physicsthreadhandle.join();
  }
  // 回收 ROS 线程
  if (ros2_spin_thread.joinable()) {
      ros2_spin_thread.join();
  }

  // D. 最后，安全删除 MuJoCo 数据
  // 此时没有任何线程在运行，删除是安全的
  if (d) {
      mj_deleteData(d);
      d = nullptr;
  }
  if (m) {
      mj_deleteModel(m);
      m = nullptr;
  }

  return 0;
}
