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
  # Common MSG
  "msg/Common/Controller/PidControllerTrace.msg"
  "msg/Common/Obersver/FnEstimatorTrace.msg"

  # High Level Control MSG
  "msg/HighLevelControl/Sport/WheelLegged/BodyWorldFrame.msg"
  "msg/HighLevelControl/Sport/WheelLegged/FiveLinkJointFrame.msg"
  "msg/HighLevelControl/Sport/WheelLegged/FiveLinkForceFrame.msg"
  "msg/HighLevelControl/Sport/WheelLegged/LegVmcJointFrame.msg"
  "msg/HighLevelControl/Sport/WheelLegged/LegVmcForceFrame.msg"
  "msg/HighLevelControl/Sport/WheelLegged/LqrControllerTrace.msg"
  "msg/HighLevelControl/Sport/WheelLegged/LegControllerTrace.msg"
  "msg/HighLevelControl/Sport/WheelLegged/StateEstimatorTrace.msg"

  "msg/HighLevelControl/Sport/ChassisCtrl.msg"
  "msg/HighLevelControl/Sport/ChassisState.msg"

  "msg/HighLevelControl/JointCmd.msg"
  "msg/HighLevelControl/JointState.msg"
  "msg/HighLevelControl/ChassisJointCmd.msg"
  "msg/HighLevelControl/ChassisJointState.msg"
  "msg/HighLevelControl/GimbalJointCmd.msg"
  "msg/HighLevelControl/GimbalJointState.msg"
  "msg/HighLevelControl/ShootJointCmd.msg"
  "msg/HighLevelControl/ShootJointState.msg"
  # Low Level Control MSG
  "msg/LowLevelControl/DJI/DjiMotorCmd.msg"
  "msg/LowLevelControl/DJI/DjiMotorState.msg"
  "msg/LowLevelControl/KTECH/KtechMotorCmd.msg"
  "msg/LowLevelControl/KTECH/KtechMotorState.msg"
  "msg/LowLevelControl/Motorevo/MotorevoMotorCmd.msg"
  "msg/LowLevelControl/Motorevo/MotorevoMotorState.msg"

  "msg/LowLevelControl/ChassisMotorCmd.msg"
  "msg/LowLevelControl/ChassisMotorState.msg"
  "msg/LowLevelControl/GimbalMotorCmd.msg"
  "msg/LowLevelControl/GimbalMotorState.msg"
  "msg/LowLevelControl/ShootMotorCmd.msg"
  "msg/LowLevelControl/ShootMotorState.msg"

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
