/**
 * @file chassis_config_manage.hpp
 * @brief 完整的底盘配置管理器 - 包含所有控制器Config
 */

#ifndef CHASSIS_CONFIG_MANAGER_HPP
#define CHASSIS_CONFIG_MANAGER_HPP

#include "base_config_manager.hpp"
#include "Controller/PIDController.hpp"
#include "Controller/LQRController.hpp"
#include "Controller/LegController.hpp"
#include "Controller/RotateController.hpp"
#include "Controller/StateEstimator.hpp"
#include "Model/WheelLeggedRobot.hpp"

namespace config_manager {

/**
* @brief 完整的底盘配置管理器
* 
* 包含所有控制器的Config类型:
* - LQRController::Config
* - LegController::Config
* - RotateController::Config
* - StateEstimator::Config
* - WheelLeggedRobot::Config
*/
class ChassisConfigManager : public BaseConfigManager {
public:
    /**
    * @brief 控制器参数集合
    * 
    * 直接使用各控制器的Config类型
    */
    struct ControllerParams {
        controller::LQRController<6, 2, 3>::Config lqr;
        controller::LegController::Config leg_control;
        controller::RotateController::Config rotate;
        controller::StateEstimator::Config state_estimator;
        
        float control_frequency = 333.33f;
    };
    
    /**
    * @brief 机器人模型参数
    * 
    * 直接使用WheelLeggedRobot::Config
    */
    using RobotModelParams = robot::WheelLeggedRobot::Config;
    
    /**
    * @brief 构造函数
    */
    explicit ChassisConfigManager(rclcpp::Node* node);
    
    // ==================== 实现基类接口 ====================
    
    bool loadModelParameters(const std::string& yaml_path) override;
    void declareParameters() override;
    void loadControllerParameters() override;
    void printConfiguration() const override;
    
    // ==================== 访问器 ====================
    
    const RobotModelParams& getModelParams() const { return model_params_; }
    const ControllerParams& getControllerParams() const { return controller_params_; }

private:
    RobotModelParams model_params_;
    ControllerParams controller_params_;
    
    void parseLegDynamicsTable(const YAML::Node& node);
};

} // namespace config_manager

#endif // CHASSIS_CONFIG_MANAGER_HPP