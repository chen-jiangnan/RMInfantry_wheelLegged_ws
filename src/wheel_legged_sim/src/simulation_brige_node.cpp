// mujoco_sim_node.cpp
// 职责：只负责 ROS2 通信（Node 类定义）
// 不包含任何 MuJoCo / GLFW 头文件
// main() 在 mujoco_sim.cpp 里

#include "rclcpp/rclcpp.hpp"
#include "wheel_legged_sim/mujoco_global.hpp"
#include "wheel_legged_sim/param.h"

#include "wheel_legged_msgs/msg/joint_cmds.hpp"
#include "wheel_legged_msgs/msg/joint_states.hpp"
#include "wheel_legged_msgs/msg/imu_state.hpp"

class MujocoSimNode : public rclcpp::Node {
public:
    explicit MujocoSimNode(const std::string& name) : Node(name) {
        RCLCPP_INFO(get_logger(), "%s 节点已启动", name.c_str());

        // ── ROS2 参数（由 launch 文件传入）──
        declare_parameter("robot",                   "cyclBot");
        declare_parameter("robot_scene",             "scene.xml");
        declare_parameter("enable_elastic_band",     false);
        declare_parameter("print_scene_information", false);
        declare_parameter("band_attached_link",      0);

        // 同步到 param::config，供 PhysicsLoop 读取
        param::config.enable_elastic_band     = get_parameter("enable_elastic_band").as_bool();
        param::config.print_scene_information = get_parameter("print_scene_information").as_bool();
        param::config.band_attached_link      = get_parameter("band_attached_link").as_int();

        // ── 发布者 ──
        chassis_state_pub_ = create_publisher<wheel_legged_msgs::msg::JointStates>(
            "lowlevel/chassisJointState", 10);
        gimbal_state_pub_  = create_publisher<wheel_legged_msgs::msg::JointStates>(
            "lowlevel/gimbalJointState", 10);
        shoot_state_pub_   = create_publisher<wheel_legged_msgs::msg::JointStates>(
            "lowlevel/shootJointState", 10);
        imu_pub_           = create_publisher<wheel_legged_msgs::msg::IMUState>(
            "lowlevel/imuState", 10);


        // ── 订阅者：写 g_sim_data.motor_cmd（atomic，线程安全）──
        chassis_cmd_sub_ = create_subscription<wheel_legged_msgs::msg::JointCmds>(
            "lowlevel/chassisJointCmd", 10,
            [](wheel_legged_msgs::msg::JointCmds::SharedPtr msg) {
                int n_chassis = g_sim_data.num_chassis.load();      // ← 动态读
                for (size_t i = 0;
                     i < std::min(msg->joint_cmds.size(), (size_t)n_chassis); ++i) {
                    const auto& jc = msg->joint_cmds[i];
                    g_sim_data.motor_cmd[i].set(jc.mode, jc.q, jc.dq, jc.tau, jc.kp, jc.kd);
                }
            });

        gimbal_cmd_sub_ = create_subscription<wheel_legged_msgs::msg::JointCmds>(
            "lowlevel/gimbalJointCmd", 10,
            [](wheel_legged_msgs::msg::JointCmds::SharedPtr msg) {
                int n_chassis = g_sim_data.num_chassis.load();      // ← 动态读
                int n_gimbal = g_sim_data.num_gimbal.load();        // ← 动态读
                for (size_t i = 0;
                     i < std::min(msg->joint_cmds.size(), (size_t)n_gimbal); ++i) {
                    const auto& jc = msg->joint_cmds[i];
                    g_sim_data.motor_cmd[n_chassis + i].set(
                        jc.mode, jc.q, jc.dq, jc.tau, jc.kp, jc.kd);
                }
            });

        shoot_cmd_sub_ = create_subscription<wheel_legged_msgs::msg::JointCmds>(
            "lowlevel/shootJointCmd", 10,
            [](wheel_legged_msgs::msg::JointCmds::SharedPtr msg) {
                int n_chassis = g_sim_data.num_chassis.load();      // ← 动态读
                int n_gimbal = g_sim_data.num_gimbal.load();        // ← 动态读
                int n_shoot = g_sim_data.num_shoot.load();          // ← 动态读
                for (size_t i = 0;
                     i < std::min(msg->joint_cmds.size(), (size_t)n_shoot); ++i) {
                    const auto& jc = msg->joint_cmds[i];
                    g_sim_data.motor_cmd[n_chassis + n_gimbal + i].set(
                        jc.mode, jc.q, jc.dq, jc.tau, jc.kp, jc.kd);
                }
            });

        // ── 状态发布定时器（500Hz）──
        state_pub_timer_ = create_wall_timer(
            std::chrono::milliseconds(2),
            [this] { publishAll(); });
    }

    ~MujocoSimNode() override {
        state_pub_timer_.reset();
    }

    std::string getRobot()      const { return get_parameter("robot").as_string(); }
    std::string getRobotScene() const { return get_parameter("robot_scene").as_string(); }

private:
    void publishAll() {
        if (!g_bridge_ready.load(std::memory_order_acquire)) return;

        int n_chassis = g_sim_data.num_chassis.load();
        int n_gimbal  = g_sim_data.num_gimbal.load();
        int n_shoot   = g_sim_data.num_shoot.load();
    
        /* 底盘（始终发布）*/
        if (n_chassis > 0) {
            auto msg = wheel_legged_msgs::msg::JointStates();
            msg.joint_states.resize(n_chassis);
            for (int i = 0; i < n_chassis; ++i) {
                auto& s = g_sim_data.motor_state[i];
                double q = s.q.load(std::memory_order_relaxed);
                double wrapped = std::remainder(q, 25.0);
                msg.joint_states[i].q = static_cast<float>(wrapped);
                msg.joint_states[i].dq  = static_cast<float>(s.dq.load(std::memory_order_relaxed));
                msg.joint_states[i].tau = static_cast<float>(s.tau.load(std::memory_order_relaxed));
            }
            chassis_state_pub_->publish(msg);
        }
    
        /* 云台（有才发布）*/
        if (n_gimbal > 0) {
            auto msg = wheel_legged_msgs::msg::JointStates();
            msg.joint_states.resize(n_gimbal);
            for (int i = 0; i < n_gimbal; ++i) {
                auto& s = g_sim_data.motor_state[n_chassis + i];
                msg.joint_states[i].q   = static_cast<float>(s.q.load(std::memory_order_relaxed));
                msg.joint_states[i].dq  = static_cast<float>(s.dq.load(std::memory_order_relaxed));
                msg.joint_states[i].tau = static_cast<float>(s.tau.load(std::memory_order_relaxed));
            }
            gimbal_state_pub_->publish(msg);
        }
    
        /* 射击（有才发布）*/
        if (n_shoot > 0) {
            auto msg = wheel_legged_msgs::msg::JointStates();
            msg.joint_states.resize(n_shoot);
            for (int i = 0; i < n_shoot; ++i) {
                auto& s = g_sim_data.motor_state[n_chassis + n_gimbal + i];
                msg.joint_states[i].q   = static_cast<float>(s.q.load(std::memory_order_relaxed));
                msg.joint_states[i].dq  = static_cast<float>(s.dq.load(std::memory_order_relaxed));
                msg.joint_states[i].tau = static_cast<float>(s.tau.load(std::memory_order_relaxed));
            }
            shoot_state_pub_->publish(msg);
        }

        /* IMU */
        {
            auto msg = wheel_legged_msgs::msg::IMUState();
            msg.header.stamp    = now();
            msg.header.frame_id = "chassis_imu";
            double q[4], gyro[3], acc[3], rpy[3];
            g_sim_data.chassis_imu.read(q, gyro, acc, rpy);
            for (int i = 0; i < 4; ++i) msg.quaternion[i]    = q[i];
            for (int i = 0; i < 3; ++i) {
                msg.gyroscope[i]     = gyro[i];
                msg.accelerometer[i] = acc[i];
                msg.rpy[i]           = rpy[i];
            }
            imu_pub_->publish(msg);
        }
    }

    rclcpp::Publisher<wheel_legged_msgs::msg::JointStates>::SharedPtr   chassis_state_pub_;
    rclcpp::Publisher<wheel_legged_msgs::msg::JointStates>::SharedPtr   gimbal_state_pub_;
    rclcpp::Publisher<wheel_legged_msgs::msg::JointStates>::SharedPtr   shoot_state_pub_;
    rclcpp::Publisher<wheel_legged_msgs::msg::IMUState>::SharedPtr      imu_pub_;

    rclcpp::Subscription<wheel_legged_msgs::msg::JointCmds>::SharedPtr  chassis_cmd_sub_;
    rclcpp::Subscription<wheel_legged_msgs::msg::JointCmds>::SharedPtr  gimbal_cmd_sub_;
    rclcpp::Subscription<wheel_legged_msgs::msg::JointCmds>::SharedPtr  shoot_cmd_sub_;

    rclcpp::TimerBase::SharedPtr state_pub_timer_;
};

/* ================================================================
 *  工厂函数（供 mujoco_sim.cpp 的 main() 调用）
 *  避免 main 直接使用 MujocoSimNode 完整类型
 * ================================================================ */
std::shared_ptr<rclcpp::Node> createMujocoSimNode(const std::string& name) {
    return std::make_shared<MujocoSimNode>(name);
}

std::string getNodeRobot(std::shared_ptr<rclcpp::Node> node) {
    return std::dynamic_pointer_cast<MujocoSimNode>(node)->getRobot();
}

std::string getNodeRobotScene(std::shared_ptr<rclcpp::Node> node) {
    return std::dynamic_pointer_cast<MujocoSimNode>(node)->getRobotScene();
}