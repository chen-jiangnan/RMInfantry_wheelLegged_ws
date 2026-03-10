#!/bin/bash
set -e

# 默认值
BAG_DIR="./bags"
BAG_NAME="record_$(date +%Y%m%d_%H%M%S)"

# 解析参数
while [[ $# -gt 0 ]]; do
  case $1 in
    --dir)
      BAG_DIR="$2"
      shift 2
      ;;
    --name)
      BAG_NAME="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument $1"
      exit 1
      ;;
  esac
done

# 创建目录
mkdir -p "$BAG_DIR"

echo "保存路径: $BAG_DIR/$BAG_NAME"
echo "开始录制... (Ctrl+C 停止)"
echo "回放: ros2 bag play ./bags/record_20240101_120000"
echo "查看 bag 信息: ros2 bag info ./bags/record_20240101_120000"
# source workspace
source install/setup.bash

# 录制
ros2 bag record \
  /highlevel/chassisCtrl \
  /highlevel/chassisState \
  /joy \
  /lowlevel/chassisJointCmd \
  /lowlevel/chassisJointState \
  /lowlevel/imuState \
  -o "$BAG_DIR/$BAG_NAME"