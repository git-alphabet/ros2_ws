#!/bin/bash
# 开启错误检查：e(遇错即停), u(禁止未定义变量), o pipefail(管道错误检查)
set -euo pipefail

# 显卡离屏渲染配置（保持原样）
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# --- 路径修正逻辑 ---
# 1. 获取脚本所在目录 (例如: /home/user/ros2_ws/scripts)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 2. 修正：WS_DIR 应该是脚本目录的上一级 (例如: /home/user/ros2_ws)
WS_DIR="$(dirname "$SCRIPT_DIR")"

# 定义安装环境路径
ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
OVERLAY_SETUP=${OVERLAY_SETUP:-$WS_DIR/install/setup.bash}

echo "[start_gazebo.sh] Workspace root: $WS_DIR"

# Source ROS 和 Overlay 环境
set +u  # 临时关闭未定义变量检查，因为 ROS 的 setup 脚本内部经常有未定义变量
if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
else
    echo "错误: 找不到系统 ROS 环境 $ROS_SETUP"
    exit 1
fi

if [ -f "$OVERLAY_SETUP" ]; then
    source "$OVERLAY_SETUP"
else
    echo "错误: 找不到工作空间环境 $OVERLAY_SETUP"
    echo "请先在终端运行: colcon build"
    exit 1
fi
set -u

# 运行 Gazebo 仿真
GAZEBO_CMD=${GAZEBO_CMD:-"ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"}

echo "[start_gazebo.sh] Starting Gazebo simulation world..."
# 使用 exec 可以让脚本进程直接被仿真进程替换，方便 Ctrl+C 彻底关闭
$GAZEBO_CMD

echo "[start_gazebo.sh] Gazebo exited."