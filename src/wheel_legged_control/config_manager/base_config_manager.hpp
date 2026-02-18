/**
 * @file base_config_manager.hpp
 * @brief 配置管理器基类
 */

#ifndef BASE_CONFIG_MANAGER_HPP
#define BASE_CONFIG_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <functional>
#include <vector>

namespace config_manager {

/**
* @brief 配置管理器基类
* 
* 提供统一的配置管理接口:
* - 模型参数: 从YAML加载（静态，不可运行时修改）
* - 控制器参数: ROS2参数（动态，可运行时修改）
*/
class BaseConfigManager {
public:
    explicit BaseConfigManager(rclcpp::Node* node);
    virtual ~BaseConfigManager() = default;
    
    // ==================== 纯虚函数 (子类必须实现) ====================
    
    /**
    * @brief 加载模型参数（从YAML文件）
    * @param yaml_path YAML文件路径（相对或绝对）
    * @return 是否成功
    */
    virtual bool loadModelParameters(const std::string& yaml_path) = 0;
    
    /**
    * @brief 声明ROS2参数
    */
    virtual void declareParameters() = 0;
    
    /**
    * @brief 加载控制器参数（从ROS2参数服务器）
    */
    virtual void loadControllerParameters() = 0;
    
    /**
    * @brief 打印配置摘要
    */
    virtual void printConfiguration() const = 0;
    
    // ==================== 通用功能 ====================
    
    /**
    * @brief 设置参数变化回调
    * @param callback 回调函数
    */
    void setParameterCallback(
        std::function<void(const std::vector<rclcpp::Parameter>&)> callback
    );

protected:
    rclcpp::Node* node_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    
    // ==================== 辅助函数 ====================
    
    /**
    * @brief 获取package的share目录路径
    * @param package_name package名称
    * @return 完整路径
    */
    std::string getPackageSharePath(const std::string& package_name);
    
    /**
    * @brief 解析配置文件路径（支持相对路径和绝对路径）
    * @param path 路径
    * @param package_name package名称
    * @return 完整路径
    */
    std::string resolveConfigPath(const std::string& path, const std::string& package_name);
    
    /**
    * @brief 加载YAML文件
    * @param file_path 文件路径
    * @return YAML节点
    */
    YAML::Node loadYAMLFile(const std::string& file_path);
    
    /**
    * @brief 参数变化回调处理
    */
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters
    );
    
    // 用户回调
    std::function<void(const std::vector<rclcpp::Parameter>&)> user_callback_;
};

} // namespace config_manager

#endif // BASE_CONFIG_MANAGER_HPP