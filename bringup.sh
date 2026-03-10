#!/bin/bash
set -e

# 默认值
ROBOT_TYPE="seriBot"
ENV_MODE="sim"

# 解析命令行参数
while [[ $# -gt 0 ]]; do
  case $1 in
    --robot)
      ROBOT_TYPE="$2"
      shift 2
      ;;
    --env)
      ENV_MODE="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument $1"
      exit 1
      ;;
  esac
done

echo "Robot type: $ROBOT_TYPE"
echo "Run mode: $ENV_MODE"

# source workspace
source install/setup.bash

# 启动
ros2 launch wheel_legged_launch bringup.launch.py \
    robot_type:=$ROBOT_TYPE env_mode:=$ENV_MODE