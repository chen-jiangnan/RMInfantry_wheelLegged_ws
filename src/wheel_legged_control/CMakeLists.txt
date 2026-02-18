cmake_minimum_required(VERSION 3.8)
project(wheel_legged_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(wheel_legged_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)


set(src_files
  "common/Filter/kalman_filter.cpp"
  "common/Model/WheelLeggedRobot.cpp"
  "config_manager/base_config_manager.cpp"
  "config_manager/chassis_config_manager.cpp"
  "interfaces/joint_cmd_interface.cpp"
  "interfaces/joint_state_interface.cpp"
  "fsm/jointFSM.cpp"
)
# ================chassis_control=========================
add_executable(chassis_control 
  src/chassis_control.cpp
  ${src_files}
)
target_include_directories(chassis_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/common>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/config_manager>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/fsm>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/interfaces>
  $<INSTALL_INTERFACE:include>)
target_compile_features(chassis_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  chassis_control
  "rclcpp"
  "wheel_legged_msgs"
  "sensor_msgs"
)
target_link_libraries(chassis_control 
  Eigen3::Eigen
  yaml-cpp 
)

install(TARGETS chassis_control
  DESTINATION lib/${PROJECT_NAME})

# ================gimbal_control=========================
add_executable(gimbal_control
  src/gimbal_control.cpp
  # ${src_files}
)
target_include_directories(gimbal_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gimbal_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  gimbal_control
  "rclcpp"
  # "wheel_legged_msgs"
  # "sensor_msgs"
)
# target_link_libraries(gimbal_control 
#   Eigen3::Eigen
#   yaml-cpp
# )

install(TARGETS gimbal_control
  DESTINATION lib/${PROJECT_NAME})

# ================shoot_control=========================
add_executable(shoot_control
  src/shoot_control.cpp
  # ${src_files}
)
target_include_directories(shoot_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(shoot_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  shoot_control
  "rclcpp"
  # "wheel_legged_msgs"
  # "sensor_msgs"
)
# target_link_libraries(shoot_control 
#   Eigen3::Eigen
#   yaml-cpp
# )

install(TARGETS shoot_control
  DESTINATION lib/${PROJECT_NAME})


# ================user_control=========================
add_executable(user_control
  src/user_control.cpp
  ${src_files}
)
target_include_directories(user_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/common>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/config_manager>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/fsm>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/interfaces>
  $<INSTALL_INTERFACE:include>)
target_compile_features(user_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  user_control
  "rclcpp"
  "wheel_legged_msgs"
  "sensor_msgs"
)
target_link_libraries(user_control 
  Eigen3::Eigen
  yaml-cpp
)

install(TARGETS user_control
  DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
