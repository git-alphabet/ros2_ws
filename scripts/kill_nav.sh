#!/bin/bash
# 清理导航相关残留进程
# 用途：仿真/实车导航崩溃或强退后，快速清除所有残留进程，避免重启时端口冲突、话题重复发布等问题
#
# 用法：
#   bash scripts/kill_nav.sh              # 清理所有导航残留
#   DRY_RUN=1 bash scripts/kill_nav.sh   # 只打印，不实际 kill（预览模式）
set -o pipefail

DRY_RUN="${DRY_RUN:-0}"

_kill() {
    local desc="$1"
    local pattern="$2"
    local pids
    pids=$(pgrep -f "$pattern" 2>/dev/null || true)
    if [[ -z "$pids" ]]; then
        echo "  [跳过] 未找到: $desc"
        return
    fi
    if [[ "$DRY_RUN" == "1" ]]; then
        echo "  [DRY] 将 kill: $desc → PIDs: $pids"
    else
        echo "  [kill] $desc → PIDs: $pids"
        # shellcheck disable=SC2086
        kill $pids 2>/dev/null || true
    fi
}

echo "========================================"
echo " kill_nav.sh  导航残留进程清理"
[[ "$DRY_RUN" == "1" ]] && echo " 模式: DRY_RUN（预览，不实际 kill）"
echo "========================================"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ── 1. 取消 Nav2 当前导航目标（有超时，不会卡住）──────────────────
echo ""
echo "[1/4] 取消 Nav2 导航目标..."

# source 在子 shell 执行，任何 setup.bash 内部错误都不影响本脚本
_ros2_cancel() {
    local ros_distro="${ROS_DISTRO:-humble}"
    # shellcheck disable=SC1090
    source "/opt/ros/${ros_distro}/setup.bash" 2>/dev/null || true
    local ws_setup="${WS_DIR}/install/setup.bash"
    [[ -f "${ws_setup}" ]] && source "${ws_setup}" 2>/dev/null || true
    ros2 service call /navigate_to_pose/_action/cancel_goal \
        action_msgs/srv/CancelGoal "{}" \
        --timeout 1 2>/dev/null
}

if [[ "$DRY_RUN" != "1" ]]; then
    if _ros2_cancel; then
        echo "  [ok] Nav2 导航目标已取消"
    else
        echo "  [跳过] Nav2 服务不可达（正常，若导航未启动）"
    fi
else
    echo "  [DRY] 将 call: /navigate_to_pose/_action/cancel_goal"
fi

# ── 2. 清理测试发布脚本残留 ────────────────────────────────────────
echo ""
echo "[2/4] 清理测试话题发布残留..."
_kill "pub_test_behavior_tree.sh"   "pub_test_behavior_tree.sh"
_kill "ros2 pub /robot_status"      "ros2 topic pub /robot_status"
_kill "ros2 pub /game_status"       "ros2 topic pub /game_status"

# ── 3. 清理导航 launch 进程 ────────────────────────────────────────
echo ""
echo "[3/4] 清理导航 launch 进程..."
_kill "rm_navigation_simulation_launch"   "rm_navigation_simulation_launch"
_kill "rm_navigation_reality_launch"      "rm_navigation_reality_launch"
_kill "nav2_bringup"                      "nav2_bringup"
_kill "nav2_lifecycle_manager"            "nav2_lifecycle_manager"
_kill "bt_navigator"                      "bt_navigator"
_kill "controller_server"                 "controller_server"
_kill "planner_server"                    "planner_server"
_kill "behavior_server"                   "behavior_server"
_kill "costmap_filter_info_server"        "costmap_filter_info_server"
_kill "map_server"                        "map_server"
_kill "amcl"                              "amcl"
_kill "point_lio"                         "pointlio_mapping"

# ── 4. 清理仿真/传感器 launch 进程 ────────────────────────────────
echo ""
echo "[4/4] 清理仿真与传感器进程..."
_kill "bringup_sim.launch"            "bringup_sim.launch"
_kill "gz_sim / ign"                  "gz sim|ign gazebo"
_kill "rm_navigation_launch (BT决策)" "rm_navigation_launch"
_kill "loam_interface"                "loam_interface_node"
_kill "sensor_scan_generation"        "sensor_scan_generation"
_kill "small_gicp_relocalization"     "small_gicp_relocalization"
_kill "livox_ros_driver2"             "livox_ros_driver2_node"
_kill "mid360_driver"                 "mid360_driver_node"

echo ""
echo "========================================"
echo " 清理完成"
[[ "$DRY_RUN" == "1" ]] && echo " （DRY_RUN 模式，未实际 kill 任何进程）"
echo "========================================"
