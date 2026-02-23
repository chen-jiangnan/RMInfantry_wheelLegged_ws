cmake_minimum_required(VERSION 3.8)
project(wheel_legged_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 依赖 ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(glfw3 REQUIRED)
find_package(fmt REQUIRED)
find_package(wheel_legged_msgs REQUIRED)
find_package(wheel_legged_interfaces REQUIRED)  # ← 新增，移除 yaml-cpp / boost

# ── MuJoCo simulate 源文件 ──
file(GLOB SIM_SRC
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc
  src/lodepng/lodepng.cpp
)

# ── 可执行文件（两个 cpp 编译到同一目标）──
add_executable(mujoco_sim
  ${SIM_SRC}
  src/mujoco_sim.cpp              # MuJoCo 生命周期 + PhysicsLoop，定义全局变量
  src/simulation_brige_node.cpp   # ROS2 通信 + main()，extern 引用全局变量
)

# ── include 路径 ──
target_include_directories(mujoco_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/mujoco/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/mujoco/simulate>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/lodepng>
  $<INSTALL_INTERFACE:include>
)

# ── ROS2 ament 依赖 ──
ament_target_dependencies(mujoco_sim
  rclcpp
  ament_index_cpp
  wheel_legged_msgs
  wheel_legged_interfaces  # ← 新增
)

# ── 自定义库目标链接 ──
target_link_libraries(mujoco_sim
  wheel_legged_interfaces::wheel_legged_interfaces  # ← 新增
  pthread
  ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/lib/libmujoco.so
  glfw
  fmt
  # yaml-cpp      ← 移除，改用 ROS2 参数
  # boost_program_options  ← 移除，改用 ROS2 参数
)

# ── 宏定义 ──
target_compile_definitions(mujoco_sim PUBLIC
  PACKAGE_NAME="${PROJECT_NAME}"
)

# ── RPATH（运行时找到 libmujoco.so 和插件）──
set_target_properties(mujoco_sim PROPERTIES
  BUILD_RPATH   "$ORIGIN/mujoco;$ORIGIN/mujoco_plugin"
  INSTALL_RPATH "$ORIGIN/mujoco;$ORIGIN/mujoco_plugin"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# ── 安装 ──
install(TARGETS mujoco_sim
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY mujoco/lib/
  DESTINATION lib/${PROJECT_NAME}/mujoco
)

install(DIRECTORY mujoco/bin/mujoco_plugin/
  DESTINATION lib/${PROJECT_NAME}/mujoco_plugin
  FILES_MATCHING PATTERN "*.so"
)

# config.yaml 不再需要安装，改由 launch 文件的 parameters 维护
# install(FILES config.yaml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()