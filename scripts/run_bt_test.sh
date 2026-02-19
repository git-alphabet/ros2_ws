#!/bin/bash
# 一体化测试: 后台启动BT节点 + 前台运行测试 + 收集日志
set -e

WS_DIR="/home/abc/rm_code/2026_1_21/ros2_ws"
cd "$WS_DIR"
source install/setup.bash

BT_LOG="/tmp/bt_test_output.log"
rm -f "$BT_LOG"

echo ">>> [1/4] 启动 BT 节点 (后台)..."
ros2 run rm_behavior_tree rm_behavior_tree \
  --ros-args -p style:="${WS_DIR}/src/rm_behavior_tree/rm_behavior_tree/config/rmul_2026.xml" \
  > "$BT_LOG" 2>&1 &
BT_PID=$!
echo "    BT PID = $BT_PID"

# 等待 BT 启动
sleep 4
if ! kill -0 $BT_PID 2>/dev/null; then
  echo "!!! BT 节点启动失败，日志:"
  cat "$BT_LOG"
  exit 1
fi
echo "    BT 节点启动成功"

echo ""
echo ">>> [2/4] 运行测试脚本..."
python3 "${WS_DIR}/scripts/test_dead_respawn.py" 2>&1
TEST_EXIT=$?
echo "    测试脚本退出码: $TEST_EXIT"

echo ""
echo ">>> [3/4] 等待 BT 处理剩余消息..."
sleep 3

echo ""
echo ">>> [4/4] BT 节点完整日志输出:"
echo "=================================================================="
cat "$BT_LOG"
echo "=================================================================="

echo ""
echo ">>> 清理: 杀死 BT 节点 (PID=$BT_PID)"
kill $BT_PID 2>/dev/null || true
wait $BT_PID 2>/dev/null || true
echo ">>> 测试完成"
