#!/usr/bin/env bash
set -e

# ================== 基本配置 ==================
WS_DIR=$(cd "$(dirname "$0")" && pwd)
BUILD_TYPE=Release
JOBS=$(nproc)

# ================== 颜色输出 ==================
GREEN="\033[32m"
YELLOW="\033[33m"
RED="\033[31m"
NC="\033[0m"

echo -e "${GREEN}=== ROS2 Workspace Build Script ===${NC}"
echo "Workspace: $WS_DIR"
echo "Build type: $BUILD_TYPE"
echo "Jobs: $JOBS"

cd "$WS_DIR"

# ================== 清理可选 ==================
if [[ "$1" == "clean" ]]; then
  echo -e "${YELLOW}Cleaning build/install/log...${NC}"
  rm -rf build install log
  unset AMENT_PREFIX_PATH
  unset CMAKE_PREFIX_PATH
  unset COLCON_PREFIX_PATH
  echo -e "${YELLOW}Resetting ROS environment...${NC}"
  source /opt/ros/humble/setup.bash
fi

# ================== Source ROS ==================
if [ -z "$ROS_DISTRO" ]; then
  echo -e "${RED}ROS not sourced. Sourcing humble...${NC}"
  source /opt/ros/humble/setup.bash
fi

# ================== 编译 ==================
echo -e "${GREEN}Building workspace...${NC}"
colcon build \
  --packages-select wheel_legged_msgs \
  --parallel-workers $JOBS \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon build \
  --packages-select wheel_legged_sim \
  --parallel-workers $JOBS \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon build \
  --packages-select wheel_legged_hw \
  --parallel-workers $JOBS \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon build \
  --packages-select wheel_legged_description \
  --symlink-install \
  --parallel-workers $JOBS
colcon build \
  --packages-select wheel_legged_control \
  --parallel-workers $JOBS \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon build \
  --packages-select wheel_legged_launch \
  --symlink-install \
  --parallel-workers $JOBS \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON


echo -e "${GREEN}Build finished successfully ✅${NC}"

# # ================== Source overlay ==================
# echo -e "${GREEN}Sourcing install/setup.bash${NC}"
# source install/setup.bash
