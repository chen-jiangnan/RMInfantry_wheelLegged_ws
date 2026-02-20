cmake_minimum_required(VERSION 3.8)
project(wheel_legged_hw)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ==================== 第三方库设置 ====================
# 设置第三方库路径
set(USB2FDCAN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/usb2fdcan/include)
set(USB2FDCAN_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/usb2fdcan/lib)

# 查找 libdm_device.so
find_library(DM_DEVICE_LIB
  NAMES dm_device libdm_device
  PATHS ${USB2FDCAN_LIB_DIR}
  NO_DEFAULT_PATH
)

if(NOT DM_DEVICE_LIB)
  message(FATAL_ERROR "无法找到 libdm_device.so,请确认文件在: ${USB2FDCAN_LIB_DIR}")
else()
  message(STATUS "找到 libdm_device.so: ${DM_DEVICE_LIB}")
endif()

# ================hardware_brige_node=========================
add_executable(hardware_brige_node
  src/hardware_brige_node.cpp
)

target_include_directories(hardware_brige_node PUBLIC
  # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/devices>
  $<BUILD_INTERFACE:${USB2FDCAN_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hardware_brige_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hardware_brige_node
  "rclcpp"
  # "wheel_legged_msgs"
  # "sensor_msgs"
)

# 链接第三方库（使用find_library找到的路径）
target_link_libraries(hardware_brige_node
  ${DM_DEVICE_LIB}
  usb-1.0
)

# ==================== 安装 ====================
# 安装可执行文件
install(TARGETS hardware_brige_node
  DESTINATION lib/${PROJECT_NAME}
)

# # 安装MuJoCo 动态库
# install(DIRECTORY ${DM_DEVICE_LIB}
# DESTINATION lib/${PROJECT_NAME}/${DM_DEVICE_LIB})

# 安装第三方.so（重要：让运行时能找到）
install(FILES ${DM_DEVICE_LIB}
  DESTINATION lib
)

# # 安装头文件
# install(DIRECTORY include/
#   DESTINATION include
# )

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
