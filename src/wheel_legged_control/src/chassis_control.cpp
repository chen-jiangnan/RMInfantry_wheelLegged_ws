#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "chassis_config_manager.hpp"
#include "Controller/LegController.hpp"
#include "Controller/LQRController.hpp"
#include "Controller/RotateController.hpp"
#include "Controller/StateEstimator.hpp"
#include "Model/WheelLeggedRobot.hpp"
#include "joint_fsm.hpp"
#include "wheel_legged_interfaces/imu_state_interface.hpp"
#include "wheel_legged_interfaces/joint_state_interface.hpp"
#include "wheel_legged_interfaces/joint_cmd_interface.hpp"
#include "wheel_legged_msgs/msg/chassis_state.hpp"
#include <array>
#include <memory>
#include "chassis_ctrl_interface.hpp"
#include "chassis_state_interface.hpp"

using namespace robot;
using namespace controller;
using namespace config_manager;
using namespace wheel_legged_interfaces;

class ChassisControlNode : public rclcpp::Node {
public:
    explicit ChassisControlNode(const std::string& name) : Node(name) {
        RCLCPP_INFO(get_logger(), "%s 节点已启动", name.c_str());

        // ── 配置管理器 ──
        config_manager_ = std::make_unique<ChassisConfigManager>(this);
        config_manager_->declareParameters();
        std::string model_path = get_parameter("model_config_path").as_string();
        if (!config_manager_->loadModelParameters(model_path))
            throw std::runtime_error("模型参数加载失败");
        config_manager_->loadControllerParameters();
        config_manager_->printConfiguration();
        config_manager_->setParameterCallback(
            std::bind(&ChassisControlNode::onParamChanged, this, std::placeholders::_1));

        // ── 接口对象 ──
        imu_state_    = std::make_unique<IMUStateInterface>();
        joints_state_ = std::make_unique<JointStateInterface>(6, JointStateInterface::MODE_IDLE);
        joints_cmd_   = std::make_unique<JointCmdInterface>(6, JointCmdInterface::MODE_IDLE);
        chassis_ctrl_ = std::make_unique<ChassisCtrlInterface>();
        chassis_state_= std::make_unique<ChassisStateInterface>();
        chassis_state_->bind(&robot_, &leg_controller_, &lqr_controller_,
                             &rotate_controller_, &state_estimator_);

        // ── FSM 初始化 ──
        initFSM();

        // ── ROS2 发布/订阅 ──
        imu_state_sub_ = create_subscription<wheel_legged_msgs::msg::IMUState>(
            "lowlevel/imuState", 10,
            [this](wheel_legged_msgs::msg::IMUState::SharedPtr msg) {
                imu_state_->fromMsg(*msg);
            });

        joints_state_sub_ = create_subscription<wheel_legged_msgs::msg::JointStates>(
            "lowlevel/chassisJointState", 10,
            [this](wheel_legged_msgs::msg::JointStates::SharedPtr msg) {
                joints_state_->fromMsgs(*msg);
            });

        chassis_ctrl_sub_ = create_subscription<wheel_legged_msgs::msg::ChassisCtrl>(
            "highlevel/chassisCtrl", 10,
            [this](wheel_legged_msgs::msg::ChassisCtrl::SharedPtr msg) {
                // FSM 状态切换
                auto it = kFSMTriggerMap.find(msg->joint_fsm_mode);
                if (it != kFSMTriggerMap.end()) {
                    fsm_->trigger(it->second);
                }
                chassis_ctrl_->fromMsg(*msg);
                // chassis_ctrl_->print();
            });

        joints_cmd_pub_    = create_publisher<wheel_legged_msgs::msg::JointCmds>(
            "lowlevel/chassisJointCmd", 10);
        chassis_state_pub_ = create_publisher<wheel_legged_msgs::msg::ChassisState>(
            "highlevel/chassisState", 10);

        // ── 定时器 ──
        joint_cmd_timer_ = create_wall_timer(
            std::chrono::milliseconds(1),
            [this] { joints_cmd_pub_->publish(joints_cmd_->toMsgs()); });

        chassis_state_timer_ = create_wall_timer(
            std::chrono::milliseconds(1),
            [this] { chassis_state_pub_->publish(chassis_state_->toMsg()); });
        // debug_timer_ = create_wall_timer(
        //     std::chrono::milliseconds(100),
        //     [this] { robot_.printModelStateInfo(); lqr_controller_.printDebugInfo(); });


        // ── 控制线程 ──
        control_thread_ = std::thread(&ChassisControlNode::controlLoop, this);
    }

    ~ChassisControlNode() override {
        if (control_thread_.joinable()) control_thread_.join();
    }

private:
    // ================================================================
    //  FSM 初始化
    //  onEnter：进入状态时执行一次（初始化/清理）
    //  onExit ：离开状态时执行一次（清理）
    //  switch ：controlLoop 里每帧持续执行
    // ================================================================
    void initFSM() {
        fsm_ = std::make_unique<JointFSM>(event_);

        auto& jfsm_cfg = config_manager_->getControllerParams().jfsm;
        if (!jfsm_cfg.require_cali) {
          event_.cali = true;
          RCLCPP_INFO(get_logger(), "[FSM] 无需校准, cali 直接置位");
      }

        // ── onEnter：进入时的一次性初始化 ──────────────────────
        fsm_->onEnter(JFSMode::ZEROTAU, [this] {
            RCLCPP_INFO(get_logger(), "[FSM] → ZEROTAU");
        });

        fsm_->onEnter(JFSMode::DAMPING, [this] {
            RCLCPP_INFO(get_logger(), "[FSM] → DAMPING");
        });

        fsm_->onEnter(JFSMode::CALI, [this] {
            RCLCPP_INFO(get_logger(), "[FSM] → CALI");
        });

        fsm_->onEnter(JFSMode::RESET, [this] {
            RCLCPP_INFO(get_logger(), "[FSM] → RESET");
        });

        fsm_->onEnter(JFSMode::READY, [this] {
            RCLCPP_INFO(get_logger(), "[FSM] → READY");
            // 进入 READY 前先确保控制器状态干净
            // （通常从 RESET 来，RESET 的 onExit 已经清过了，这里保险起见再清一次）
            clearControllers();
        });

        // ── onExit：离开时的清理 ────────────────────────────────
        fsm_->onExit(JFSMode::READY, [this] {
            // 退出 READY 时重置控制器，防止残留积分/历史影响下一次进入
            RCLCPP_INFO(get_logger(), "[FSM] READY exit → 重置控制器");
            clearControllers();
        });

        fsm_->onExit(JFSMode::RESET, [this] {
            RCLCPP_INFO(get_logger(), "[FSM] RESET exit → 重置控制器");
            clearControllers();
        });
    }

    // ── 统一清理入口 ────────────────────────────────────────────
    void clearControllers() {
        leg_controller_.clear();
        lqr_controller_.clear();
        rotate_controller_.clear();
        state_estimator_.clear();
    }

    // ── 参数热更新 ──────────────────────────────────────────────
    void onParamChanged(const std::vector<rclcpp::Parameter>&) {
        params_changed_.store(true);
    }

    void reCfgControllers() {
        std::lock_guard<std::mutex> lock(param_mutex_);
        auto& p = config_manager_->getControllerParams();
        lqr_controller_.setConfig(p.lqr);
        leg_controller_.setConfig(p.leg_control);
        rotate_controller_.setConfig(p.rotate);
        state_estimator_.setConfig(p.state_estimator);
        RCLCPP_INFO(get_logger(), "控制器参数已更新");
    }

    // ================================================================
    //  关节控制实现
    //  这些函数在 controlLoop 的 switch 里每帧调用
    // ================================================================
    void jointZeroTau() {
        for (int i = 0; i < 6; ++i) {
            joints_cmd_->setMode(i, JointCmdInterface::MODE_IDLE);
            joints_cmd_->setPosition(i, 0);
            joints_cmd_->setVelocity(i, 0);
            joints_cmd_->setEffort(i, 0);
            joints_cmd_->setKp(i, 0);
            joints_cmd_->setKd(i, 0);
        }
    }

    void jointDamping() {
        for (int i = 0; i < 6; ++i) {
            joints_cmd_->setMode(i, JointCmdInterface::MODE_MIT);
            joints_cmd_->setPosition(i, 0);
            joints_cmd_->setVelocity(i, 0);
            joints_cmd_->setEffort(i, 0);
            joints_cmd_->setKp(i, 0);
            joints_cmd_->setKd(i, 3);
        }
    }

    void jointCali() {
        // TODO: 驱动髋关节到限位，设置零点偏移
    }

    void jointReset() {
        // TODO: 多阶段腿部复位
    }

    void jointReady(double dt) {
        robot_.updateBodyPose(imu_state_->getRPY(),
                              imu_state_->getGyroscope(),
                              imu_state_->getAccelerometer());
        robot_.updateAllJointState(*joints_state_);

        std::array<float, 2> phi1, phi4, phi0, L0;
        phi1[0] = robot_.getHipJoints()[0].q;
        phi4[0] = robot_.getHipJoints()[1].q;
        phi1[1] = robot_.getHipJoints()[2].q;
        phi4[1] = robot_.getHipJoints()[3].q;
        robot_.forwardKinematics(phi1, phi4, phi0, L0, true);

        std::array<float, 2> tA, tE, Tp, F;
        tA[0] = robot_.getHipJoints()[0].tau;
        tE[0] = robot_.getHipJoints()[1].tau;
        tA[1] = robot_.getHipJoints()[2].tau;
        tE[1] = robot_.getHipJoints()[3].tau;
        robot_.forwardDynamics(tA, tE, Tp, F, true);

        std::array<float, 2> alpha, theta;
        alpha[0] = robot_.getVMCJointFrames()[0].alpha;
        alpha[1] = robot_.getVMCJointFrames()[1].alpha;
        theta[0] = robot_.getVMCJointFrames()[0].theta;
        theta[1] = robot_.getVMCJointFrames()[1].theta;
        state_estimator_.update(alpha, theta, L0);

        std::array<float, 2> w_ecd;
        w_ecd[0] = robot_.getWheelJoints()[0].dq;
        w_ecd[1] = robot_.getWheelJoints()[1].dq;
        state_estimator_.estimateVelocity(
            w_ecd,
            robot_.getBodyWorldFrame().a[0],
            robot_.getBodyWorldFrame().rpy[1],
            robot_.getConfig().wheel_link[0].lengthOrRadius);

        state_estimator_.estimateForce(
            F, Tp,
            robot_.getBodyWorldFrame().a[2] - 9.8f,
            robot_.getConfig().wheel_link[0].mass);

        LQRController6x2::StateVector xd;
        xd.setZero();
        std::array<LQRController6x2::StateVector, 2> x;
        for (int i = 0; i < 2; ++i) {
            x[i](0) = robot_.getVMCJointFrames()[i].theta;
            x[i](1) = (robot_.getVMCJointFrames()[i].theta_history[0]
                      -robot_.getVMCJointFrames()[i].theta_history[1]) / dt;
            x[i](2) = state_estimator_.getVelocityEstimate().x_filter;
            x[i](3) = state_estimator_.getVelocityEstimate().v_filter;
            x[i](4) = -robot_.getBodyWorldFrame().rpy[1];
            x[i](5) = -robot_.getBodyWorldFrame().w[1];
            lqr_controller_.updateKFormLegLength(i, L0[i]);
        }
        auto lqr_u = lqr_controller_.calculate(xd, x);

        std::array<float, 2> target_L0 = {0.20f, 0.20f};
        auto leg_u = leg_controller_.calculate(
            target_L0, L0, 0,
            robot_.getBodyWorldFrame().rpy[0],
            theta, phi0,
            robot_.getConfig().body_link.mass * G);
        
        std::array<float, 2> rotate_u = {0, 0};
        rotate_u[0] = rotate_controller_.calculate(0, robot_.getBodyWorldFrame().w[2]);
        rotate_u[1] = -rotate_u[0];

        std::array<float, 2> set_Tp, set_F, set_T, set_tA, set_tE;
        for (int i = 0; i < 2; ++i) {
            set_Tp[i] = lqr_u[i](1) + leg_u.compensation_tp[i];
            set_F[i]  = leg_u.F[i];
            set_T[i]  = lqr_u[i](0) + rotate_u[i];
        }
        robot_.inverseDynamics(set_Tp, set_F, set_tA, set_tE);

        // 力矩限幅
        for (int i = 0; i < 2; ++i) {
            set_tA[i] = std::clamp(set_tA[i], -50.0f, 50.0f);
            set_tE[i] = std::clamp(set_tE[i], -50.0f, 50.0f);
            set_T[i]  = std::clamp(set_T[i],   -5.0f,  5.0f);
        }

        std::array<float, 6> jp = {0};
        std::array<float, 6> jv = {0};
        // std::array<float, 6> jt = {0};
        std::array<float, 6> jt = {
            set_tA[0], set_tE[0], set_T[0],
            set_tA[1], set_tE[1], set_T[1]
        };

        for (int i = 0; i < 6; ++i) {
            jp[i] = (robot_.getConfig().joints[i].invert_pos     ? -1 : 1)
                    * (jp[i] - robot_.getConfig().joints[i].pos_offset);
            jv[i] = (robot_.getConfig().joints[i].invert_vel     ? -1 : 1) * jv[i];
            jt[i] = (robot_.getConfig().joints[i].invert_torque  ? -1 : 1) * jt[i];

            joints_cmd_->setMode(i, JointCmdInterface::MODE_MIT);
            joints_cmd_->setPosition(i, jp[i]);
            joints_cmd_->setVelocity(i, jv[i]);
            joints_cmd_->setEffort(i, jt[i]);
            joints_cmd_->setKp(i, 0);
            joints_cmd_->setKd(i, 0);
        }
    }

    // ================================================================
    //  控制主循环
    // ================================================================
    void controlLoop() {
        // 初始化模型和控制器
        robot_.setConfig(config_manager_->getModelParams());
        auto& p = config_manager_->getControllerParams();
        lqr_controller_.setConfig(p.lqr);
        leg_controller_.setConfig(p.leg_control);
        rotate_controller_.setConfig(p.rotate);
        state_estimator_.setConfig(p.state_estimator);
        RCLCPP_INFO(get_logger(), "控制器初始化完成");

        const double dt = 1.0 / p.control_frequency;

        while (rclcpp::ok()) {
            // 热更新参数
            if (params_changed_.load()) {
                reCfgControllers();
                params_changed_.store(false);
            }

            // 自动错误检测（过流/通信丢失等由外部写 event_.error）
            fsm_->checkAutoTransitions();

            // 每帧根据当前状态持续执行对应控制
            // onEnter/onExit 不在这里触发，只在 trigger/checkAutoTransitions 切换时触发
            switch (fsm_->current()) {
                case JFSMode::ZEROTAU:  jointZeroTau();   break;
                case JFSMode::DAMPING:  jointDamping();   break;
                case JFSMode::CALI:     jointCali();      break;
                case JFSMode::RESET:    jointReset();     break;
                case JFSMode::READY:    jointReady(dt);   break;
            }

            std::this_thread::sleep_for(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::duration<double>(dt)));
        }
    }

    // ── 配置 ──
    std::unique_ptr<ChassisConfigManager> config_manager_;
    std::atomic<bool> params_changed_{false};
    std::mutex param_mutex_;

    // ── 接口 ──
    std::unique_ptr<IMUStateInterface>    imu_state_;
    std::unique_ptr<JointStateInterface>  joints_state_;
    std::unique_ptr<JointCmdInterface>    joints_cmd_;
    std::unique_ptr<ChassisCtrlInterface> chassis_ctrl_;
    std::unique_ptr<ChassisStateInterface>chassis_state_;

    // ── FSM ──
    FSMEvent                    event_;
    std::unique_ptr<JointFSM>   fsm_;

    // ── ROS2 ──
    rclcpp::Subscription<wheel_legged_msgs::msg::IMUState>::SharedPtr     imu_state_sub_;
    rclcpp::Subscription<wheel_legged_msgs::msg::JointStates>::SharedPtr  joints_state_sub_;
    rclcpp::Subscription<wheel_legged_msgs::msg::ChassisCtrl>::SharedPtr  chassis_ctrl_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                fsm_trigger_sub_;
    rclcpp::Publisher<wheel_legged_msgs::msg::JointCmds>::SharedPtr       joints_cmd_pub_;
    rclcpp::Publisher<wheel_legged_msgs::msg::ChassisState>::SharedPtr    chassis_state_pub_;
    rclcpp::TimerBase::SharedPtr joint_cmd_timer_;
    rclcpp::TimerBase::SharedPtr chassis_state_timer_;

    // rclcpp::TimerBase::SharedPtr debug_timer_;


    // ── 模型和控制器 ──
    WheelLeggedRobot    robot_;
    LQRController6x2    lqr_controller_;
    LegController       leg_controller_;
    RotateController    rotate_controller_;
    StateEstimator      state_estimator_;
    std::thread         control_thread_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChassisControlNode>("chassis_control_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}