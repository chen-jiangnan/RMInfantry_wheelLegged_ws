#pragma once

#include <filesystem>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace param {

/* ================================================================
 *  SimConfig：从 ROS2 参数服务器读取，不再依赖 yaml-cpp
 *
 *  launch 文件里配置：
 *    Node(
 *      parameters=[{
 *        'robot':                   'cyclBot',
 *        'robot_scene':             'scene.xml',
 *        'enable_elastic_band':     False,
 *        'print_scene_information': False,
 *        'band_attached_link':      0,
 *      }]
 *    )
 * ================================================================ */
struct SimConfig {
    std::string            robot;
    std::filesystem::path  robot_scene;
    std::string            robot_file_path;
    bool                   enable_elastic_band     = false;
    bool                   print_scene_information = false;
    int                    band_attached_link       = 0;

    void load_from_node(rclcpp::Node* node) {
        robot       = node->get_parameter("robot").as_string();
        robot_scene = node->get_parameter("robot_scene").as_string();
        enable_elastic_band     = node->get_parameter("enable_elastic_band").as_bool();
        print_scene_information = node->get_parameter("print_scene_information").as_bool();
        band_attached_link      = node->get_parameter("band_attached_link").as_int();
    }
};

inline SimConfig config;

} // namespace param