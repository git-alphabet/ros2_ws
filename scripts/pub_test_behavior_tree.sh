#!/bin/bash
# 发布仿真测试用的裁判系统话题
# 用于在没有真实裁判系统时模拟比赛状态，消除行为树的 WARN 日志
#
# 用法：
#   bash scripts/pub_test_status.sh                            # 默认：比赛进行中, 血量=400
#   GAME_PROGRESS=0 bash scripts/pub_test_status.sh            # 非比赛阶段（BT 会回家）
#   CURRENT_HP=150  bash scripts/pub_test_status.sh            # 模拟低血量（<200 回补给区）
#   MODE=respawn PRE_DELAY=15 bash scripts/pub_test_status.sh  # 复活沿测试：先正常导航15s → 战亡3s → 复活hp=80 → BT自动回补给区
#   START_DELAY=5 bash scripts/pub_test_status.sh             # 先停滞5s（非比赛阶段）再切换为比赛进行中，BT才开始走
#   MODE=kill_and_revive bash scripts/pub_test_status.sh       # 新开终端随时随地触发：立刻发hp=0，10s后切换hp=80，全程game_status不断
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
    ros2 topic pub /robot_status rm_decision_interfaces/msg/RMUL \
        "{current_hp: 0}" \
        --rate "${PUB_RATE}" &
    PID_DEAD=$!
    trap "kill ${PID_DEAD} 2>/dev/null; exit 0" INT TERM

    echo "[pub_test_status][kill_and_revive] >>> hp=0 发布中，等待 ${DEAD_DURATION}s ..."
    sleep "${DEAD_DURATION}"
    kill "${PID_DEAD}" 2>/dev/null
    wait "${PID_DEAD}" 2>/dev/null || true

    # 切换为复活血量，持续发布
    echo "[pub_test_status][kill_and_revive] >>> 切换为 hp=${REVIVE_HP}，BT 应触发复活沿导航回补给区 ..."
    ros2 topic pub /robot_status rm_decision_interfaces/msg/RMUL \
        "{current_hp: ${REVIVE_HP}}" \
        --rate "${PUB_RATE}" &
    PID_REVIVE=$!
    trap "kill ${PID_REVIVE} 2>/dev/null; exit 0" INT TERM

    wait "${PID_REVIVE}"
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
    ros2 topic pub /game_status rm_decision_interfaces/msg/RMUL \
        "{game_progress: ${GAME_PROGRESS}, stage_remain_time: ${STAGE_REMAIN_TIME}}" \
        --rate "${PUB_RATE}" &
    PID_GAME=$!
    trap "kill ${PID_GAME} 2>/dev/null; exit 0" INT TERM

    # 预备阶段：正常血量，让机器人先导航去目标点
    if [[ "${PRE_DELAY}" -gt 0 ]]; then
        ros2 topic pub /robot_status rm_decision_interfaces/msg/RMUL \
            "{current_hp: ${CURRENT_HP}}" \
            --rate "${PUB_RATE}" &
        PID_PRE=$!
        trap "kill ${PID_GAME} ${PID_PRE} 2>/dev/null; exit 0" INT TERM
        echo "[pub_test_status][respawn] >>> hp=${CURRENT_HP} 发布中，等待 ${PRE_DELAY}s ..."
        sleep "${PRE_DELAY}"
        kill "${PID_PRE}" 2>/dev/null
        wait "${PID_PRE}" 2>/dev/null || true
    fi

    # 第一阶段：hp=0
    ros2 topic pub /robot_status rm_decision_interfaces/msg/RMUL \
        "{current_hp: 0}" \
        --rate "${PUB_RATE}" &
    PID_DEAD=$!
    trap "kill ${PID_GAME} ${PID_DEAD} 2>/dev/null; exit 0" INT TERM

    echo "[pub_test_status][respawn] >>> hp=0 发布中，等待 ${DEAD_DURATION}s ..."
    sleep "${DEAD_DURATION}"
    kill "${PID_DEAD}" 2>/dev/null
    wait "${PID_DEAD}" 2>/dev/null || true

    # 第二阶段：切换为复活血量
    echo "[pub_test_status][respawn] >>> 切换为 hp=${REVIVE_HP}，观察 BT 是否触发复活沿导航回补给区 ..."
    ros2 topic pub /robot_status rm_decision_interfaces/msg/RMUL \
        "{current_hp: ${REVIVE_HP}}" \
        --rate "${PUB_RATE}" &
    PID_REVIVE=$!
    trap "kill ${PID_GAME} ${PID_REVIVE} 2>/dev/null; exit 0" INT TERM

    wait "${PID_REVIVE}"
    exit 0
fi

# ── 正常发布模式 ──────────────────────────────────────────────────────────────
echo "[pub_test_status] game_progress=${GAME_PROGRESS}, stage_remain_time=${STAGE_REMAIN_TIME}, current_hp=${CURRENT_HP}, rate=${PUB_RATE}Hz"
[[ "${START_DELAY}" -gt 0 ]] && echo "[pub_test_status] 先停滞 ${START_DELAY}s（game_progress=0），再切换为比赛进行中"
echo "[pub_test_status] 按 Ctrl+C 停止发布"
echo ""

# 全程持续发布 robot_status
ros2 topic pub /robot_status rm_decision_interfaces/msg/RMUL \
    "{current_hp: ${CURRENT_HP}}" \
    --rate "${PUB_RATE}" &
PID_ROBOT=$!
trap "kill ${PID_ROBOT} 2>/dev/null; exit 0" INT TERM

# START_DELAY 阶段：先发 game_progress=0，BT 处于非比赛阶段不动
if [[ "${START_DELAY}" -gt 0 ]]; then
    ros2 topic pub /game_status rm_decision_interfaces/msg/RMUL \
        "{game_progress: 0, stage_remain_time: ${STAGE_REMAIN_TIME}}" \
        --rate "${PUB_RATE}" &
    PID_WAIT=$!
    trap "kill ${PID_ROBOT} ${PID_WAIT} 2>/dev/null; exit 0" INT TERM
    echo "[pub_test_status] >>> 停滞中，等待 ${START_DELAY}s ..."
    sleep "${START_DELAY}"
    kill "${PID_WAIT}" 2>/dev/null
    wait "${PID_WAIT}" 2>/dev/null || true
    echo "[pub_test_status] >>> 切换为 game_progress=${GAME_PROGRESS}，BT 开始执行"
fi

# 发布正式 game_status
ros2 topic pub /game_status rm_decision_interfaces/msg/RMUL \
    "{game_progress: ${GAME_PROGRESS}, stage_remain_time: ${STAGE_REMAIN_TIME}}" \
    --rate "${PUB_RATE}" &
PID_GAME=$!

# 等待任意一个退出（Ctrl+C 会同时终止两个）
trap "kill ${PID_GAME} ${PID_ROBOT} 2>/dev/null; exit 0" INT TERM
wait ${PID_GAME} ${PID_ROBOT}
