/**
 * @file chassis_config_manager.cpp
 * @brief 底盘配置管理器实现 - 使用控制器Config类型
 */

#include "chassis_config_manager.hpp"
#include <iostream>
#include <iomanip>

namespace config_manager {

ChassisConfigManager::ChassisConfigManager(rclcpp::Node* node)
    : BaseConfigManager(node)
{
}

// ==================== 声明ROS2参数 ====================

void ChassisConfigManager::declareParameters() {
    
    RCLCPP_INFO(node_->get_logger(), "声明底盘控制器参数...");
    
    // 模型配置路径
    node_->declare_parameter("model_config_path", "");
    
    // 控制频率
    node_->declare_parameter("control_frequency", 333.33);
    
    // ==================== LQR参数 ====================
    
    node_->declare_parameter("lqr.k_matrix.rows", 2);
    node_->declare_parameter("lqr.k_matrix.cols", 6);
    node_->declare_parameter("lqr.k_matrix.data", std::vector<double>(12, 0.0));
    node_->declare_parameter("lqr.leg_fitting.enabled", false);
    node_->declare_parameter("lqr.leg_fitting.min_length", 0.13);
    node_->declare_parameter("lqr.leg_fitting.max_length", 0.40);
    node_->declare_parameter("lqr.leg_fitting.coeffs", std::vector<double>(36, 0.0));
    
    // ==================== 腿控制参数 ====================
    
    // 弹簧阻尼
    node_->declare_parameter("leg_control.spring_damping.kp", 1000.0);
    node_->declare_parameter("leg_control.spring_damping.ki", 0.0);
    node_->declare_parameter("leg_control.spring_damping.kd", 200.0);
    node_->declare_parameter("leg_control.spring_damping.max_out", 300.0);
    node_->declare_parameter("leg_control.spring_damping.max_iout", 300.0);
    
    // Roll补偿
    node_->declare_parameter("leg_control.roll_compensation.kp", 50.0);
    node_->declare_parameter("leg_control.roll_compensation.ki", 0.0);
    node_->declare_parameter("leg_control.roll_compensation.kd", 10.0);
    node_->declare_parameter("leg_control.roll_compensation.max_out", 300.0);
    node_->declare_parameter("leg_control.roll_compensation.max_iout", 300.0);
    
    // Phi0补偿
    node_->declare_parameter("leg_control.phi0_compensation.kp", 5.0);
    node_->declare_parameter("leg_control.phi0_compensation.ki", 0.0);
    node_->declare_parameter("leg_control.phi0_compensation.kd", 1.0);
    node_->declare_parameter("leg_control.phi0_compensation.max_out", 10.0);
    node_->declare_parameter("leg_control.phi0_compensation.max_iout", 10.0);
    
    // ==================== 旋转控制参数 ====================
    
    node_->declare_parameter("rotate_control.kp", 2.0);
    node_->declare_parameter("rotate_control.ki", 0.0);
    node_->declare_parameter("rotate_control.kd", 0.5);
    node_->declare_parameter("rotate_control.max_out", 3.0);
    node_->declare_parameter("rotate_control.max_iout", 3.0);
    
    // ==================== 状态估计器参数 ====================
    
    node_->declare_parameter("state_estimator.dt", 0.003);
    node_->declare_parameter("state_estimator.kalman_filter.q0", 0.5);
    node_->declare_parameter("state_estimator.kalman_filter.q1", 0.5);
    node_->declare_parameter("state_estimator.kalman_filter.r0", 100.0);
    node_->declare_parameter("state_estimator.kalman_filter.r1", 100.0);
    
    RCLCPP_INFO(node_->get_logger(), "参数声明完成");
}

// ==================== 加载控制器参数 ====================

void ChassisConfigManager::loadControllerParameters() {
    
    RCLCPP_INFO(node_->get_logger(), "加载底盘控制器参数...");
    
    // 控制频率
    controller_params_.control_frequency = 
        node_->get_parameter("control_frequency").as_double();
    
    // ========== LQR ==========
    
    auto k_vec = node_->get_parameter("lqr.k_matrix.data").as_double_array();
    if (k_vec.size() == 12) {
        std::array<float, 12> k_array;
        for (size_t i = 0; i < 12; ++i) {
            k_array[i] = static_cast<float>(k_vec[i]);
        }
        controller_params_.lqr.setKMatrix(0, k_array.data());
        controller_params_.lqr.setKMatrix(1, k_array.data());
    }
    
    controller_params_.lqr.leg_fitting_enabled = 
        node_->get_parameter("lqr.leg_fitting.enabled").as_bool();
    controller_params_.lqr.min_leg_length = 
        node_->get_parameter("lqr.leg_fitting.min_length").as_double();
    controller_params_.lqr.max_leg_length = 
        node_->get_parameter("lqr.leg_fitting.max_length").as_double();
    
    auto fit_vec = node_->get_parameter("lqr.leg_fitting.coeffs").as_double_array();
    if (fit_vec.size() == 36) {
        for (size_t i = 0; i < 12; ++i) {
            for (size_t j = 0; j < 3; ++j) {
                controller_params_.lqr.fit_coeffs[i][j] = 
                    static_cast<float>(fit_vec[i * 3 + j]);
            }
        }
    }
    
    // ========== 腿控制 - 弹簧阻尼 ==========
    
    controller_params_.leg_control.spring_damping.pid.kp = 
        node_->get_parameter("leg_control.spring_damping.kp").as_double();
    controller_params_.leg_control.spring_damping.pid.ki = 
        node_->get_parameter("leg_control.spring_damping.ki").as_double();
    controller_params_.leg_control.spring_damping.pid.kd = 
        node_->get_parameter("leg_control.spring_damping.kd").as_double();
    controller_params_.leg_control.spring_damping.max_out = 
        node_->get_parameter("leg_control.spring_damping.max_out").as_double();
    controller_params_.leg_control.spring_damping.max_iout = 
        node_->get_parameter("leg_control.spring_damping.max_iout").as_double();
    
    // 更新PID配置的max_out和max_iout
    controller_params_.leg_control.spring_damping.pid.max_out = 
        controller_params_.leg_control.spring_damping.max_out;
    controller_params_.leg_control.spring_damping.pid.max_iout = 
        controller_params_.leg_control.spring_damping.max_iout;
    
    // ========== 腿控制 - Roll补偿 ==========
    
    controller_params_.leg_control.roll_compensation.pid.kp = 
        node_->get_parameter("leg_control.roll_compensation.kp").as_double();
    controller_params_.leg_control.roll_compensation.pid.ki = 
        node_->get_parameter("leg_control.roll_compensation.ki").as_double();
    controller_params_.leg_control.roll_compensation.pid.kd = 
        node_->get_parameter("leg_control.roll_compensation.kd").as_double();
    controller_params_.leg_control.roll_compensation.max_out = 
        node_->get_parameter("leg_control.roll_compensation.max_out").as_double();
    controller_params_.leg_control.roll_compensation.max_iout = 
        node_->get_parameter("leg_control.roll_compensation.max_iout").as_double();
    
    controller_params_.leg_control.roll_compensation.pid.max_out = 
        controller_params_.leg_control.roll_compensation.max_out;
    controller_params_.leg_control.roll_compensation.pid.max_iout = 
        controller_params_.leg_control.roll_compensation.max_iout;
    
    // ========== 腿控制 - Phi0补偿 ==========
    
    controller_params_.leg_control.phi0_compensation.pid.kp = 
        node_->get_parameter("leg_control.phi0_compensation.kp").as_double();
    controller_params_.leg_control.phi0_compensation.pid.ki = 
        node_->get_parameter("leg_control.phi0_compensation.ki").as_double();
    controller_params_.leg_control.phi0_compensation.pid.kd = 
        node_->get_parameter("leg_control.phi0_compensation.kd").as_double();
    controller_params_.leg_control.phi0_compensation.max_out = 
        node_->get_parameter("leg_control.phi0_compensation.max_out").as_double();
    controller_params_.leg_control.phi0_compensation.max_iout = 
        node_->get_parameter("leg_control.phi0_compensation.max_iout").as_double();
    
    controller_params_.leg_control.phi0_compensation.pid.max_out = 
        controller_params_.leg_control.phi0_compensation.max_out;
    controller_params_.leg_control.phi0_compensation.pid.max_iout = 
        controller_params_.leg_control.phi0_compensation.max_iout;
    
    // ========== 旋转控制 ==========
    
    controller_params_.rotate.pid.kp = 
        node_->get_parameter("rotate_control.kp").as_double();
    controller_params_.rotate.pid.ki = 
        node_->get_parameter("rotate_control.ki").as_double();
    controller_params_.rotate.pid.kd = 
        node_->get_parameter("rotate_control.kd").as_double();
    controller_params_.rotate.max_out = 
        node_->get_parameter("rotate_control.max_out").as_double();
    controller_params_.rotate.max_iout = 
        node_->get_parameter("rotate_control.max_iout").as_double();
    
    controller_params_.rotate.pid.max_out = controller_params_.rotate.max_out;
    controller_params_.rotate.pid.max_iout = controller_params_.rotate.max_iout;
    
    // ========== 状态估计器 ==========
    
    controller_params_.state_estimator.dt = 
        node_->get_parameter("state_estimator.dt").as_double();
    controller_params_.state_estimator.kalman_filter_param.Q[0] = 
        node_->get_parameter("state_estimator.kalman_filter.q0").as_double();
    controller_params_.state_estimator.kalman_filter_param.Q[1] = 
        node_->get_parameter("state_estimator.kalman_filter.q1").as_double();
    controller_params_.state_estimator.kalman_filter_param.R[0] = 
        node_->get_parameter("state_estimator.kalman_filter.r0").as_double();
    controller_params_.state_estimator.kalman_filter_param.R[1] = 
        node_->get_parameter("state_estimator.kalman_filter.r1").as_double();
    
    RCLCPP_INFO(node_->get_logger(), "底盘控制器参数加载完成");
}

// ==================== 加载模型参数 ====================

bool ChassisConfigManager::loadModelParameters(const std::string& yaml_path) {
    try {
        std::string full_path = resolveConfigPath(yaml_path, "wheel_legged_launch");
        YAML::Node config = loadYAMLFile(full_path);
        
        // 解析body
        if (config["body"]) {
            auto body = config["body"];
            model_params_.body_link.mass = body["mass"].as<float>();
            model_params_.body_link.pitch_inertia = body["pitch_inertia"].as<float>();
            model_params_.body_link.yaw_inertia = body["yaw_inertia"].as<float>();
            model_params_.body_link.l = body["com_offset"].as<float>();
            model_params_.body_link.theta_b0 = body["theta_b0"].as<float>();
        }
        
        // 解析leg_thigh
        if (config["leg_thigh"]) {
            auto thigh = config["leg_thigh"];
            for(int i = 0; i < 2; i++){
                model_params_.thigh_link[i].mass = thigh["mass"].as<float>();
                model_params_.thigh_link[i].inertia = thigh["inertia"].as<float>();
                model_params_.thigh_link[i].lengthOrRadius = thigh["length"].as<float>();
            }
        }
        
        // 解析leg_shank
        if (config["leg_shank"]) {
            auto shank = config["leg_shank"];
            for(int i = 0; i < 2; i++){
                model_params_.shank_link[i].mass = shank["mass"].as<float>();
                model_params_.shank_link[i].inertia = shank["inertia"].as<float>();
                model_params_.shank_link[i].lengthOrRadius = shank["length"].as<float>();
            }
        }
        
        // 解析wheel
        if (config["wheel"]) {
            auto wheel = config["wheel"];
            for(int i = 0; i < 2; i++){
                model_params_.wheel_link[i].mass = wheel["mass"].as<float>();
                model_params_.wheel_link[i].inertia = wheel["inertia"].as<float>();
                model_params_.wheel_link[i].lengthOrRadius = wheel["radius"].as<float>();
            }
        }
        
        // 解析geometry
        if (config["geometry"]) {
            auto geom = config["geometry"];
            model_params_.hip_joint_offset = geom["hip_joint_offset"].as<float>();
            model_params_.hip_joint_distance = geom["hip_joint_distance"].as<float>();
            model_params_.half_wheel_track = geom["half_wheel_track"].as<float>();
        }
        model_params_.updateJointOffsets();

        // 解析腿动力学表
        if (config["leg_dynamics_table"]) {
            parseLegDynamicsTable(config["leg_dynamics_table"]);
        }
        
        RCLCPP_INFO(node_->get_logger(), "底盘模型参数加载成功");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "底盘模型参数加载失败: %s", e.what());
        return false;
    }
}

void ChassisConfigManager::parseLegDynamicsTable(const YAML::Node& node) {
    if (!node["constant"] || !node["data"]) {
        return;
    }
    
    for(int i  = 0; i < 2; i++){
        model_params_.virtual_link[i].mass = node["constant"]["leg_mass"].as<float>();
    }
        
    auto data = node["data"];
    std::vector<std::array<double, 4>> table;
    for (size_t i = 0; i < data.size(); ++i) {
        std::array<float, 4> row;
        for (int j = 0; j < 4; ++j) {
            row[j] = data[i][j].as<float>();
        }
        model_params_.leg_dynamics_table.push_back(row);
    }
    
    RCLCPP_INFO(node_->get_logger(), "加载腿动力学表: %zu 个采样点", 
            model_params_.leg_dynamics_table.size());
}

// ==================== 打印配置 ====================

void ChassisConfigManager::printConfiguration() const {
    std::cout << "\n========================================" << std::endl;
    std::cout << "底盘配置" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    
    /* ==================== 机体参数 ==================== */
    std::cout << "\n[机体参数]\n";
    std::cout << "  mass          = " << model_params_.body_link.mass << " kg\n";
    std::cout << "  com_offset(l) = " << model_params_.body_link.l << " m\n";
    std::cout << "  pitch_inertia = " << model_params_.body_link.pitch_inertia << " kg*m^2\n";
    std::cout << "  yaw_inertia   = " << model_params_.body_link.yaw_inertia << " kg*m^2\n";
    std::cout << "  theta_b0      = " << model_params_.body_link.theta_b0 << " rad\n";
    /* ==================== 大腿参数 ==================== */
    std::cout << "\n[大腿参数]\n";
    std::cout << "    mass    = " << model_params_.thigh_link[0].mass << " kg\n";
    std::cout << "    length  = " << model_params_.thigh_link[0].lengthOrRadius << " m\n";
    std::cout << "    inertia = " << model_params_.thigh_link[0].inertia << " kg*m^2\n";
    /* ==================== 小腿参数 ==================== */
    std::cout << "\n[小腿参数]\n";
    std::cout << "    mass    = " << model_params_.shank_link[0].mass << " kg\n";
    std::cout << "    length  = " << model_params_.shank_link[0].lengthOrRadius << " m\n";
    std::cout << "    inertia = " << model_params_.shank_link[0].inertia << " kg*m^2\n";
    /* ==================== 轮子参数 ==================== */
    std::cout << "\n[轮子参数]\n";
    std::cout << "    mass    = " << model_params_.wheel_link[0].mass << " kg\n";
    std::cout << "    radius  = " << model_params_.wheel_link[0].lengthOrRadius << " m\n";
    std::cout << "    inertia = " << model_params_.wheel_link[0].inertia << " kg*m^2\n";
    /* ==================== 几何参数 ==================== */
    std::cout << "\n[几何参数]\n";
    std::cout << "  hip_joint_offset   = " << model_params_.hip_joint_offset << " rad\n";
    std::cout << "  hip_joint_distance = " << model_params_.hip_joint_distance << " m\n";
    std::cout << "  half_wheel_track   = " << model_params_.half_wheel_track << " m\n";
    
    // LQR
    std::cout << "\n[LQR控制器]" << std::endl;
    std::cout << "  K矩阵 (2×6):" << std::endl;
    std::cout << "  K矩阵 [left_leg]:" << std::endl;
    for (int i = 0; i < 2; ++i) {
        std::cout << "    [";
        for (int j = 0; j < 6; ++j) {
            std::cout << std::setw(10) << controller_params_.lqr.K[0](i, j);
            if (j < 5) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    std::cout << "  K矩阵 [right_leg]:" << std::endl;
    for (int i = 0; i < 2; ++i) {
        std::cout << "    [";
        for (int j = 0; j < 6; ++j) {
            std::cout << std::setw(10) << controller_params_.lqr.K[1](i, j);
            if (j < 5) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    std::cout << "  coeffs:" << std::endl;
    for (int i = 0; i < 12; ++i) {
        std::cout << "    [";
        for (int j = 0; j < 3; ++j) {
            std::cout << std::setw(10) << controller_params_.lqr.fit_coeffs[i][j];
            if (j < 2) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    std::cout << "  腿长拟合: " << (controller_params_.lqr.leg_fitting_enabled ? "启用" : "禁用") << std::endl;
    
    // 腿控制
    std::cout << "\n[腿控制器]" << std::endl;
    std::cout << "  弹簧阻尼PID: [" 
            << controller_params_.leg_control.spring_damping.pid.kp << ", "
            << controller_params_.leg_control.spring_damping.pid.ki << ", "
            << controller_params_.leg_control.spring_damping.pid.kd << "]" << std::endl;
    std::cout << "  Roll补偿PID: [" 
            << controller_params_.leg_control.roll_compensation.pid.kp << ", "
            << controller_params_.leg_control.roll_compensation.pid.ki << ", "
            << controller_params_.leg_control.roll_compensation.pid.kd << "]" << std::endl;
    
    // 旋转控制
    std::cout << "\n[旋转控制器]" << std::endl;
    std::cout << "  PID: [" 
            << controller_params_.rotate.pid.kp << ", "
            << controller_params_.rotate.pid.ki << ", "
            << controller_params_.rotate.pid.kd << "]" << std::endl;
    
    std::cout << "\n[控制频率]" << std::endl;
    std::cout << "  " << controller_params_.control_frequency << " Hz" << std::endl;
    
    std::cout << "========================================\n" << std::endl;
}

} // namespace config_manager