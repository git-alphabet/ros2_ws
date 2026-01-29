#!/usr/bin/env bash
set -euo pipefail

# 目的：仿真联调 /auto_aim_yaw 通路
# - 后台让 Gazebo 云台持续旋转（调用 scripts/spin_gimbal.sh）
# - 同时对比：
#   1) /<ns>/auto_aim_yaw
#   2) TF: chassis -> gimbal_yaw (tf2_echo)
#
# 用法：
#   ./scripts/verify_auto_aim_yaw_sim.sh [/red_standard_robot1]
# 环境变量：
#   DURATION_SEC=8        # 联调持续时长
#   YAW_STEP_RAD=0.02     # 每条 gimbal_cmd 的 yaw 增量（相对角度）
#   RATE_HZ=50            # gimbal_cmd 发送频率
#   PARENT_FRAME=chassis
#   CHILD_FRAME=gimbal_yaw
#+#+#+#+环境变量：
#   AUTO_START_SIM_PUB=1  # 如果没看到 /auto_aim_yaw，自动启动仿真发布节点
#   USE_STAMPED_MSG=true  # 自动启动节点时使用 Float32Stamped（true/false）

NS="${1:-/red_standard_robot1}"
DURATION_SEC="${DURATION_SEC:-8}"
PARENT_FRAME="${PARENT_FRAME:-chassis}"
CHILD_FRAME="${CHILD_FRAME:-gimbal_yaw}"
AUTO_START_SIM_PUB="${AUTO_START_SIM_PUB:-1}"
USE_STAMPED_MSG="${USE_STAMPED_MSG:-true}"
PREFER_CPP_PUB="${PREFER_CPP_PUB:-1}"

# 参数类型小坑：部分 rclcpp 节点把 publish_rate_hz 声明为 double，传整数会抛 InvalidParameterTypeException。
RATE_HZ_RAW="${RATE_HZ:-50}"
if [[ "${RATE_HZ_RAW}" == *.* ]]; then
  PUBLISH_RATE_HZ_PARAM="${RATE_HZ_RAW}"
else
  PUBLISH_RATE_HZ_PARAM="${RATE_HZ_RAW}.0"
fi

pids=()
cleanup() {
  for pid in "${pids[@]}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup INT TERM EXIT

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# 规范化 NS
if [[ -n "$NS" && "$NS" != /* ]]; then
  NS="/$NS"
fi

set +u
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  source "$WS_DIR/install/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
set -u

AUTO_AIM_TOPIC="${NS}/auto_aim_yaw"
GIMBAL_CMD_TOPIC="${NS}/robot_base/gimbal_cmd"

echo "[verify_auto_aim_yaw_sim.sh] NS=${NS}"
echo "[verify_auto_aim_yaw_sim.sh] duration=${DURATION_SEC}s"
echo "[verify_auto_aim_yaw_sim.sh] auto_aim=${AUTO_AIM_TOPIC}"
echo "[verify_auto_aim_yaw_sim.sh] gimbal_cmd=${GIMBAL_CMD_TOPIC}"
echo "[verify_auto_aim_yaw_sim.sh] tf2_echo ${PARENT_FRAME} -> ${CHILD_FRAME}"

AUTO_AIM_MSG_TYPE="std_msgs/msg/Float32"
USE_STAMPED_EFFECTIVE="false"

# 优先：如果系统里已经存在该 topic，就直接读出实际类型，避免“同名不同类型”导致创建 publisher 失败。
EXISTING_TYPE="$(ros2 topic type "${AUTO_AIM_TOPIC}" --spin-time 0.2 2>/dev/null || true)"
if [[ -n "${EXISTING_TYPE}" ]]; then
  AUTO_AIM_MSG_TYPE="${EXISTING_TYPE}"
  if [[ "${EXISTING_TYPE}" == "gimbal_yaw_interfaces/msg/Float32Stamped" ]]; then
    USE_STAMPED_EFFECTIVE="true"
  fi
else
  # 否则：按用户意图 + 本机接口可用性决定
  if [[ "${USE_STAMPED_MSG}" == "true" ]]; then
    if ros2 interface show gimbal_yaw_interfaces/msg/Float32Stamped >/dev/null 2>&1; then
      AUTO_AIM_MSG_TYPE="gimbal_yaw_interfaces/msg/Float32Stamped"
      USE_STAMPED_EFFECTIVE="true"
    else
      echo "[verify_auto_aim_yaw_sim.sh] INFO: gimbal_yaw_interfaces/msg/Float32Stamped 不可用，/auto_aim_yaw 将用 std_msgs/msg/Float32" >&2
      USE_STAMPED_EFFECTIVE="false"
    fi
  fi
fi

if ! ros2 topic list | grep -qx "${GIMBAL_CMD_TOPIC}"; then
  echo "[verify_auto_aim_yaw_sim.sh] ERROR: 找不到 ${GIMBAL_CMD_TOPIC}，先确认 Gazebo/robot_base 是否启动" >&2
  echo "[verify_auto_aim_yaw_sim.sh] HINT: ros2 topic list | grep gimbal_cmd" >&2
  exit 2
fi

start_auto_aim_pub_if_needed() {
  if ros2 topic list | grep -qx "${AUTO_AIM_TOPIC}"; then
    return 0
  fi

  echo "[verify_auto_aim_yaw_sim.sh] WARN: 还没有看到 ${AUTO_AIM_TOPIC}" >&2
  echo "[verify_auto_aim_yaw_sim.sh]      当前仅开 Gazebo 也没关系：脚本会启动一个 TF→/auto_aim_yaw 发布器兜底" >&2

  if [[ "${AUTO_START_SIM_PUB}" != "1" ]]; then
    return 0
  fi

  # 默认优先用 Python 兜底发布器：不依赖 C++ 编译产物，且避免某些环境下同名 topic 类型冲突。
  if [[ "${PREFER_CPP_PUB}" == "1" ]] && ros2 pkg executables gimbal_yaw_bridge 2>/dev/null | grep -q "gimbal_state_to_auto_aim_yaw"; then
    if ! ros2 node list | grep -qx "${NS}/gimbal_state_to_auto_aim_yaw"; then
      echo "[verify_auto_aim_yaw_sim.sh]      (PREFER_CPP_PUB=1) 启动 gimbal_state_to_auto_aim_yaw (C++)..." >&2
      (
        ros2 run gimbal_yaw_bridge gimbal_state_to_auto_aim_yaw \
          --ros-args \
          -r /tf:=tf -r /tf_static:=tf_static -r __ns:=${NS} \
          -p use_sim_time:=true \
          -p use_tf_source:=true \
          -p parent_frame:=${PARENT_FRAME} \
          -p child_frame:=${CHILD_FRAME} \
          -p output_topic:=auto_aim_yaw \
          -p use_stamped_msg:=${USE_STAMPED_EFFECTIVE} \
          -p publish_rate_hz:=${PUBLISH_RATE_HZ_PARAM}
      ) &
      pids+=("$!")
    fi
  else
    if ! ros2 node list | grep -qx "${NS}/tf_to_auto_aim_yaw_pub"; then
      echo "[verify_auto_aim_yaw_sim.sh]      启动 scripts/tf_to_auto_aim_yaw_pub.py (Python) 发布 /auto_aim_yaw..." >&2
      (
        python3 -u "$SCRIPT_DIR/tf_to_auto_aim_yaw_pub.py" \
          --ros-args \
          -r /tf:=tf -r /tf_static:=tf_static -r __ns:=${NS} \
          -p use_sim_time:=true \
          -p parent_frame:=${PARENT_FRAME} \
          -p child_frame:=${CHILD_FRAME} \
          -p output_topic:=auto_aim_yaw \
          -p use_stamped_msg:=${USE_STAMPED_EFFECTIVE} \
          -p publish_rate_hz:=${PUBLISH_RATE_HZ_PARAM}
      ) &
      pids+=("$!")
    fi
  fi

  # 等待 /auto_aim_yaw 出现在 topic list（最多 2s）
  for _ in $(seq 1 20); do
    if ros2 topic list | grep -qx "${AUTO_AIM_TOPIC}"; then
      return 0
    fi
    sleep 0.1
  done

  echo "[verify_auto_aim_yaw_sim.sh] ERROR: 仍未看到 ${AUTO_AIM_TOPIC}" >&2
  echo "[verify_auto_aim_yaw_sim.sh] HINT: 你可以手动跑：ros2 node list | grep -E 'auto_aim|tf_to_auto_aim'" >&2
}

start_auto_aim_pub_if_needed

echo "[verify_auto_aim_yaw_sim.sh] 1) 后台开始转云台..."
(
  # 让它静默运行；verify 脚本本身负责输出
  QUIET=1 DURATION_SEC="${DURATION_SEC}" \
  "$SCRIPT_DIR/spin_gimbal.sh" "${NS}" "${YAW_STEP_RAD:-0.02}" "${RATE_HZ:-50}"
) &
pids+=("$!")

sleep 0.5

echo "[verify_auto_aim_yaw_sim.sh] 2) echo /auto_aim_yaw（仅展示前 20 条）"
(
  # 你的 ros2 CLI 不支持 -n；这里用 timeout 控制输出时长
  timeout -s INT "${DURATION_SEC}" \
    ros2 topic echo "${AUTO_AIM_TOPIC}" "${AUTO_AIM_MSG_TYPE}" --once || true
) &
pids+=("$!")

echo "[verify_auto_aim_yaw_sim.sh] 2.1) 连续 echo /auto_aim_yaw（展示 ${DURATION_SEC}s 内输出）"
(
  timeout -s INT "${DURATION_SEC}" \
    ros2 topic echo "${AUTO_AIM_TOPIC}" "${AUTO_AIM_MSG_TYPE}" || true
) &
pids+=("$!")

sleep 0.2

echo "[verify_auto_aim_yaw_sim.sh] 3) tf2_echo（展示 ${DURATION_SEC}s 内的输出）"
(
  timeout -s INT "${DURATION_SEC}" \
    ros2 run tf2_ros tf2_echo "${PARENT_FRAME}" "${CHILD_FRAME}" -r 10 \
      --ros-args -r /tf:=tf -r /tf_static:=tf_static -r __ns:=${NS} || true
) &
pids+=("$!")

wait || true

echo "[verify_auto_aim_yaw_sim.sh] done"
