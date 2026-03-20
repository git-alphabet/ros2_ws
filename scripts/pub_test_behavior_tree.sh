#!/bin/bash
# 发布仿真测试用的裁判系统话题
# 用于在没有真实裁判系统时模拟比赛状态，消除行为树的 WARN 日志
#
# 用法：
#   bash scripts/pub_test_behavior_tree.sh                            # 默认：比赛进行中, 血量=400
#   GAME_PROGRESS=0 bash scripts/pub_test_behavior_tree.sh            # 非比赛阶段（BT 会回家）
#   CURRENT_HP=150  bash scripts/pub_test_behavior_tree.sh            # 模拟低血量（<200 回补给区）
#   MODE=respawn PRE_DELAY=60 bash scripts/pub_test_behavior_tree.sh  # 复活沿测试：先正常导航60s → 战亡3s → 复活hp=80 → BT自动回补给区
#   START_DELAY=5 bash scripts/pub_test_behavior_tree.sh             # 先停滞5s（非比赛阶段）再切换为比赛进行中，BT才开始走
#   MODE=kill_and_revive bash scripts/pub_test_behavior_tree.sh       # 新开终端随时随地触发：立刻发hp=0，10s后切换hp=80，全程game_status不断
#   MODE=set_hp SET_HP=150 bash scripts/pub_test_behavior_tree.sh      # 随时发布指定血量（<200触发回补给区），只发 /robot_status
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── 可覆盖的参数 ────────────────────────────────────────────
GAME_PROGRESS="${GAME_PROGRESS:-4}"        # 4=比赛进行中，0=未开始
STAGE_REMAIN_TIME="${STAGE_REMAIN_TIME:-180}"
CURRENT_HP="${CURRENT_HP:-400}"
PUB_RATE="${PUB_RATE:-10}"                 # Hz，建议 ≥10，防止 BT 帧间丢消息切换分支
MODE="${MODE:-normal}"                     # normal=正常发布, respawn=复活沿测试
START_DELAY="${START_DELAY:-0}"           # >0 时：先发 game_progress=0 停滞 x 秒，再切换为比赛进行中

# ── Source ROS ──────────────────────────────────────────────
ROS_DISTRO="${ROS_DISTRO:-humble}"
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1090
source "${WS_DIR}/install/setup.bash"
set -u

# ── 统一清理函数 ─────────────────────────────────────────────
# 所有后台 PID 都注册到这个数组，EXIT/INT/TERM 时统一清理
_BG_PIDS=()

cleanup() {
    echo ""
    echo "[pub_test_status] 清理中..."

    # 1. 终止所有本脚本启动的后台 ros2 topic pub 进程
    for pid in "${_BG_PIDS[@]:-}"; do
        kill "${pid}" 2>/dev/null || true
    done
    # 等待后台进程退出，避免僵尸进程
    for pid in "${_BG_PIDS[@]:-}"; do
        wait "${pid}" 2>/dev/null || true
    done

    # 2. 取消 Nav2 当前所有导航目标（超时1s，Nav2未运行时快速跳过）
    ros2 service call /navigate_to_pose/_action/cancel_goal \
        action_msgs/srv/CancelGoal "{}" \
        --timeout 1 2>/dev/null || true

    # 3. 清理残留的 ros2 topic pub 僵尸进程（防止同名进程未被捕获）
    pkill -f "ros2 topic pub /robot_status" 2>/dev/null || true
    pkill -f "ros2 topic pub /game_status"  2>/dev/null || true

    echo "[pub_test_status] 清理完成"
}

# 注册退出钩子：无论正常退出、Ctrl+C、还是 kill 都会触发
trap cleanup EXIT INT TERM

# 辅助函数：启动后台进程并自动注册 PID
bg_pub() {
    "$@" &
    _BG_PIDS+=($!)
}

# ── set_hp 模式 ──────────────────────────────────────────────────────────────────
# 随时随地可用：只发 /robot_status，不碰 /game_status
# 用途：测试低血量（<200）无条件触发回补给区的 BT 决策
# 推荐用法：
#   终端1（保持运行）: bash scripts/pub_test_behavior_tree.sh          # 维持 game_status + 正常 robot_status
#   终端2（按需触发）: SET_HP=150 MODE=set_hp bash scripts/pub_test_behavior_tree.sh  # 覆盖 robot_status，发布低血量
# 终端2 Ctrl+C 后，终端1 的正常血量自动恢复
if [[ "${MODE}" == "set_hp" ]]; then
    SET_HP="${SET_HP:-150}"   # 要发布的血量值，默认150（<200，触发回补给区）

    echo "[pub_test_status][set_hp] 发布 hp=${SET_HP}，只发 /robot_status（不干扰 /game_status）"
    echo "[pub_test_status][set_hp] BT 阈值：<200 无条件回补给区，200~300 中血量战斗逻辑，≥300 高血量"
    echo "[pub_test_status][set_hp] 按 Ctrl+C 停止（/game_status 由另一个终端的脚本维持）"
    echo ""

    bg_pub ros2 topic pub /robot_status rm_decision_interfaces/msg/RMULRob \
        "{current_hp: ${SET_HP}, x: 0.0, y: 0.0}" \
        --rate "${PUB_RATE}"

    wait
    exit 0
fi

# ── kill_and_revive 模式 ──────────────────────────────────────────────────────
# 随时随地可用：只发 /robot_status，不碰 /game_status
# 在原脚本已经跑着的情况下，Ctrl+C 原脚本后立刻执行本模式，
# game_status 短暂中断期间 BT 会回家；若不想中断，需保持原脚本运行（但会冲突）
# → 推荐做法：Ctrl+C 原脚本 → 立刻执行本模式（间隙 <1s，BT 容忍短暂丢帧）
if [[ "${MODE}" == "kill_and_revive" ]]; then
    DEAD_DURATION="${DEAD_DURATION:-10}"   # hp=0 持续秒数，默认10s
    REVIVE_HP="${REVIVE_HP:-80}"           # 复活后血量

    echo "[pub_test_status][kill_and_revive] 随时触发复活沿测试（仅发 /robot_status）"
    echo "[pub_test_status][kill_and_revive] 立即发 hp=0 持续 ${DEAD_DURATION}s → 切换为 hp=${REVIVE_HP}"
    echo "[pub_test_status][kill_and_revive] ⚠ 请先 Ctrl+C 停掉原脚本，再执行本模式，避免话题冲突"
    echo "[pub_test_status][kill_and_revive] 按 Ctrl+C 可提前终止"
    echo ""

    # 立即发 hp=0（不发 game_status，由原脚本或手动维持）
    bg_pub ros2 topic pub /robot_status rm_decision_interfaces/msg/RMULRob \
        "{current_hp: 0, x: 0.0, y: 0.0}" \
        --rate "${PUB_RATE}"
    _DEAD_PID=${_BG_PIDS[-1]}

    echo "[pub_test_status][kill_and_revive] >>> hp=0 发布中，等待 ${DEAD_DURATION}s ..."
    sleep "${DEAD_DURATION}"
    kill "${_DEAD_PID}" 2>/dev/null
    wait "${_DEAD_PID}" 2>/dev/null || true

    # 切换为复活血量，持续发布
    echo "[pub_test_status][kill_and_revive] >>> 切换为 hp=${REVIVE_HP}，BT 应触发复活沿导航回补给区 ..."
    bg_pub ros2 topic pub /robot_status rm_decision_interfaces/msg/RMULRob \
        "{current_hp: ${REVIVE_HP}, x: 0.0, y: 0.0}" \
        --rate "${PUB_RATE}"

    wait
    exit 0
fi

# ── 复活沿测试模式 ────────────────────────────────────────────────────────────
# 原理：
#   DetectRespawnAndSetRecovery 节点检测"死亡→存活"的上升沿：
#     1. hp=0 → was_dead=true
#     2. hp>0 且连续 RESPAWN_STABLE_FRAMES=2 帧 → 触发复活沿 → need_recovery=true
#     3. BT 进入 IsDeadAndDispelDebuff 子树，导航至补给区回血
#   因此测试流程：先持续发 hp=0（3秒），再切换为 hp=80
if [[ "${MODE}" == "respawn" ]]; then
    PRE_DELAY="${PRE_DELAY:-0}"            # 战亡前正常发 hp=CURRENT_HP 的秒数（0=直接战亡）
    DEAD_DURATION="${DEAD_DURATION:-3}"    # hp=0 持续秒数（>2s 即可让 was_dead 稳定置位）
    REVIVE_HP="${REVIVE_HP:-80}"           # 复活后的血量（<200 触发回补给区逻辑）

    echo "[pub_test_status][respawn] 复活沿测试模式"
    [[ "${PRE_DELAY}" -gt 0 ]] && echo "[pub_test_status][respawn] 预备阶段：先发 hp=${CURRENT_HP} 持续 ${PRE_DELAY}s（机器人正常导航去目标点）"
    echo "[pub_test_status][respawn] 第一阶段：发布 hp=0 持续 ${DEAD_DURATION}s（模拟战亡）"
    echo "[pub_test_status][respawn] 第二阶段：切换为 hp=${REVIVE_HP}（模拟复活），BT 应自动导航回补给区"
    echo "[pub_test_status][respawn] 按 Ctrl+C 可提前终止"
    echo ""

    # 持续发布 game_status（全程保持比赛进行中）
    bg_pub ros2 topic pub /game_status rm_decision_interfaces/msg/RMULRob \
        "{game_progress: ${GAME_PROGRESS}, stage_remain_time: ${STAGE_REMAIN_TIME}}" \
        --rate "${PUB_RATE}"

    # 预备阶段：正常血量，让机器人先导航去目标点
    if [[ "${PRE_DELAY}" -gt 0 ]]; then
        bg_pub ros2 topic pub /robot_status rm_decision_interfaces/msg/RMULRob \
            "{current_hp: ${CURRENT_HP}, x: 0.0, y: 0.0}" \
            --rate "${PUB_RATE}"
        _PRE_PID=${_BG_PIDS[-1]}
        echo "[pub_test_status][respawn] >>> hp=${CURRENT_HP} 发布中，等待 ${PRE_DELAY}s ..."
        sleep "${PRE_DELAY}"
        kill "${_PRE_PID}" 2>/dev/null
        wait "${_PRE_PID}" 2>/dev/null || true
    fi

    # 第一阶段：hp=0
    bg_pub ros2 topic pub /robot_status rm_decision_interfaces/msg/RMULRob \
        "{current_hp: 0, x: 0.0, y: 0.0}" \
        --rate "${PUB_RATE}"
    _DEAD_PID=${_BG_PIDS[-1]}

    echo "[pub_test_status][respawn] >>> hp=0 发布中，等待 ${DEAD_DURATION}s ..."
    sleep "${DEAD_DURATION}"
    kill "${_DEAD_PID}" 2>/dev/null
    wait "${_DEAD_PID}" 2>/dev/null || true

    # 第二阶段：切换为复活血量
    echo "[pub_test_status][respawn] >>> 切换为 hp=${REVIVE_HP}，观察 BT 是否触发复活沿导航回补给区 ..."
    bg_pub ros2 topic pub /robot_status rm_decision_interfaces/msg/RMULRob \
        "{current_hp: ${REVIVE_HP}, x: 0.0, y: 0.0}" \
        --rate "${PUB_RATE}"

    wait
    exit 0
fi

# ── 正常发布模式 ──────────────────────────────────────────────────────────────
echo "[pub_test_status] game_progress=${GAME_PROGRESS}, stage_remain_time=${STAGE_REMAIN_TIME}, current_hp=${CURRENT_HP}, rate=${PUB_RATE}Hz"
[[ "${START_DELAY}" -gt 0 ]] && echo "[pub_test_status] 先停滞 ${START_DELAY}s（game_progress=0），再切换为比赛进行中"
echo "[pub_test_status] 按 Ctrl+C 停止发布"
echo ""

# 全程持续发布 robot_status
bg_pub ros2 topic pub /robot_status rm_decision_interfaces/msg/RMULRob \
    "{current_hp: ${CURRENT_HP}, x: 0.0, y: 0.0}" \
    --rate "${PUB_RATE}"

# START_DELAY 阶段：先发 game_progress=0，BT 处于非比赛阶段不动
if [[ "${START_DELAY}" -gt 0 ]]; then
    bg_pub ros2 topic pub /game_status rm_decision_interfaces/msg/RMULRob \
        "{game_progress: 0, stage_remain_time: ${STAGE_REMAIN_TIME}}" \
        --rate "${PUB_RATE}"
    _WAIT_PID=${_BG_PIDS[-1]}
    echo "[pub_test_status] >>> 停滞中，等待 ${START_DELAY}s ..."
    sleep "${START_DELAY}"
    kill "${_WAIT_PID}" 2>/dev/null
    wait "${_WAIT_PID}" 2>/dev/null || true
    echo "[pub_test_status] >>> 切换为 game_progress=${GAME_PROGRESS}，BT 开始执行"
fi

# 发布正式 game_status
bg_pub ros2 topic pub /game_status rm_decision_interfaces/msg/RMULRob \
    "{game_progress: ${GAME_PROGRESS}, stage_remain_time: ${STAGE_REMAIN_TIME}}" \
    --rate "${PUB_RATE}"

wait
