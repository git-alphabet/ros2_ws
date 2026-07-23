#!/bin/bash
set -euo pipefail

# 导航启动脚本
# 支持 --sim 和 --reality 参数选择模式，默认使用 reality 模式

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 默认模式
MODE="reality"

# 收集额外的 launch 参数
LAUNCH_ARGS=()

# 解析参数
while [[ $# -gt 0 ]]; do
  case "$1" in
    --sim)
      MODE="sim"
      shift
      ;;
    --reality)
      MODE="reality"
      shift
      ;;
    --help|-h)
      echo "Usage: $0 [--sim|--reality] [additional launch arguments]"
      echo "  --sim       使用仿真模式"
      echo "  --reality   使用实车模式（默认）"
      echo "  --help      显示帮助信息"
      exit 0
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

# 设置环境变量
export NO_NEW_TERMINAL="${NO_NEW_TERMINAL:-1}"
export QT_FONT_DPI=192
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# 根据模式选择 launch 文件
if [ "$MODE" = "sim" ]; then
  LAUNCH_FILE="rm_navigation_simulation_launch.py"
  echo "[nav.sh] 启动仿真导航模式"
else
  LAUNCH_FILE="rm_navigation_reality_launch.py"
  echo "[nav.sh] 启动实车导航模式"
fi

# 启动 launch
exec ros2 launch gxu2026_nav_bringup "$LAUNCH_FILE" "${LAUNCH_ARGS[@]}"
