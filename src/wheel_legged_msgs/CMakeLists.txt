cmake_minimum_required(VERSION 3.8)
project(wheel_legged_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  

  # High Level Control MSG
  "msg/HighLevelControl/Common/Controller/PidControllerTrace.msg"
  "msg/HighLevelControl/Common/Controller/LegControllerTrace.msg"
  "msg/HighLevelControl/Common/Controller/LqrControllerTrace.msg"
  "msg/HighLevelControl/Common/Controller/RotateControllerTrace.msg"
  "msg/HighLevelControl/Common/Controller/StateEstimatorTrace.msg"

  "msg/HighLevelControl/Common/Obersver/KalmanFilterTrace.msg"
  "msg/HighLevelControl/Common/Obersver/FnEstimatorTrace.msg"
  "msg/HighLevelControl/Common/Obersver/XVEstimatorTrace.msg"

  "msg/HighLevelControl/Common/Model/BodyWorldFrame.msg"
  "msg/HighLevelControl/Common/Model/FiveLinkJointFrame.msg"
  "msg/HighLevelControl/Common/Model/FiveLinkForceFrame.msg"
  "msg/HighLevelControl/Common/Model/LegVmcJointFrame.msg"
  "msg/HighLevelControl/Common/Model/LegVmcForceFrame.msg"
  "msg/HighLevelControl/Common/Model/WheelLeggedRobotState.msg"

  "msg/HighLevelControl/ChassisCtrl.msg"
  "msg/HighLevelControl/ChassisState.msg"

  # Low Level Control MSG

  "msg/LowLevelControl/JointCmd.msg"
  "msg/LowLevelControl/JointState.msg"
  "msg/LowLevelControl/JointCmds.msg"
  "msg/LowLevelControl/JointStates.msg"
  "msg/LowLevelControl/IMUState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# install(DIRECTORY include/
#   DESTINATION include/
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
