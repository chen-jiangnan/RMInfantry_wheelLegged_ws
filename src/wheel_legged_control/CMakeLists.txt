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

set(src_files
  "common/Application/legControl.cpp"
  "common/Application/rotateControl.cpp"
  "common/Controllers/lqr.cpp"
  "common/Controllers/pid.cpp"
  "common/Engine/engine.cpp"
  "common/FSM/jointFSM.cpp"

)
# chassis_control
add_executable(chassis_control 
  src/chassis_control.cpp
  ${src_files}
)
target_include_directories(chassis_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/common>
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
)

install(TARGETS chassis_control
  DESTINATION lib/${PROJECT_NAME})

# wheellegged_fsm
add_executable(wheellegged_fsm 
  src/wheellegged_fsm.cpp
  ${src_files}
)
target_include_directories(wheellegged_fsm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/common>
  $<INSTALL_INTERFACE:include>)
target_compile_features(wheellegged_fsm PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  wheellegged_fsm
  "rclcpp"
  "wheel_legged_msgs"
  "sensor_msgs"
)
target_link_libraries(wheellegged_fsm 
  Eigen3::Eigen
)

install(TARGETS wheellegged_fsm
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
