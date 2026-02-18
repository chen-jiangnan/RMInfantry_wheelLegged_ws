/**
 * @file base_config_manager.cpp
 * @brief 配置管理器基类实现
 */

#include "base_config_manager.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <filesystem>

namespace config_manager {

BaseConfigManager::BaseConfigManager(rclcpp::Node* node)
    : node_(node)
    , user_callback_(nullptr)
{
}

// ==================== 参数回调 ====================

void BaseConfigManager::setParameterCallback(
    std::function<void(const std::vector<rclcpp::Parameter>&)> callback)
{
    user_callback_ = callback;
    
    auto internal_callback = [this](const std::vector<rclcpp::Parameter>& params) 
        -> rcl_interfaces::msg::SetParametersResult 
    {
        return parametersCallback(params);
    };
    
    callback_handle_ = node_->add_on_set_parameters_callback(internal_callback);
}

rcl_interfaces::msg::SetParametersResult BaseConfigManager::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters)
{
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    
    // 重新加载控制器参数
    loadControllerParameters();
    
    // 调用用户回调
    if (user_callback_) {
        user_callback_(parameters);
    }
    
    return result;
}

// ==================== 辅助函数 ====================

std::string BaseConfigManager::getPackageSharePath(const std::string& package_name) {
    try {
        return ament_index_cpp::get_package_share_directory(package_name);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), 
                    "获取package路径失败 [%s]: %s", 
                    package_name.c_str(), e.what());
        return "";
    }
}

std::string BaseConfigManager::resolveConfigPath(
    const std::string& path, 
    const std::string& package_name)
{
    // 绝对路径
    if (path.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "配置路径为空");
        return "";
    }
    
    if (path[0] == '/') {
        return path;
    }
    
    // 相对路径,从package share目录查找
    std::string package_share = getPackageSharePath(package_name);
    if (package_share.empty()) {
        return "";
    }
    
    std::filesystem::path full_path = std::filesystem::path(package_share) / path;
    return full_path.string();
}

YAML::Node BaseConfigManager::loadYAMLFile(const std::string& file_path) {
    if (!std::filesystem::exists(file_path)) {
        throw std::runtime_error("配置文件不存在: " + file_path);
    }
    
    RCLCPP_INFO(node_->get_logger(), "加载YAML文件: %s", file_path.c_str());
    
    try {
        return YAML::LoadFile(file_path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML解析失败: " + std::string(e.what()));
    }
}

} // namespace config_manager