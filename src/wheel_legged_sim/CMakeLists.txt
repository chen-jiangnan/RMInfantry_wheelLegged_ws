cmake_minimum_required(VERSION 3.8)
project(wheel_legged_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(ament_index_cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(wheel_legged_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 构建可执行文件
file(GLOB SIM_SRC
  # exclude mujoco/simulate/main.cc
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc
  src/lodepng/lodepng.cpp
)

add_executable(mujoco_sim 
  ${SIM_SRC}
  src/mujoco_sim.cpp
  # src/robot_sdk_bridge.cpp
)

# 添加路径
target_include_directories(mujoco_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/mujoco/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/mujoco/simulate>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/lodepng>
  $<INSTALL_INTERFACE:include>)

# 链接ros2依赖
ament_target_dependencies(mujoco_sim
  "rclcpp"
  "ament_index_cpp"
  "wheel_legged_msgs"
  "sensor_msgs"
)

# 定义宏
target_compile_definitions(mujoco_sim PUBLIC
  PACKAGE_NAME="${PROJECT_NAME}"
)

# 链接第三方依赖
target_link_libraries(mujoco_sim 
  pthread 
  ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/lib/libmujoco.so 
  glfw 
  yaml-cpp 
  fmt 
  boost_program_options
)

# 使安装后的可执行文件运行时能找到库和插件
set_target_properties(mujoco_sim PROPERTIES
  BUILD_RPATH "$ORIGIN/mujoco;$ORIGIN/mujoco_plugin"
  INSTALL_RPATH "$ORIGIN/mujoco;$ORIGIN/mujoco_plugin"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# 安装可执行文件
install(TARGETS mujoco_sim
  DESTINATION lib/${PROJECT_NAME})

# 安装MuJoCo 动态库
install(DIRECTORY mujoco/lib/
DESTINATION lib/${PROJECT_NAME}/mujoco)

# 安装MuJoCo plugin
install(DIRECTORY mujoco/bin/mujoco_plugin/
  DESTINATION lib/${PROJECT_NAME}/mujoco_plugin
  FILES_MATCHING PATTERN "*.so")

# 安装配置文件
install(
  FILES config.yaml
  DESTINATION share/${PROJECT_NAME}
)
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
