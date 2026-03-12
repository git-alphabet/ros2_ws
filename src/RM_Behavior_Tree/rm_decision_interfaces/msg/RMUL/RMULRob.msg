# =============================================================================
# RMULRobStatus.msg — 电控发布的裁判系统数据
# 话题: /rmul_rob_status    来源: 电控串口解析节点
# =============================================================================

std_msgs/Header header

# ── 1. 比赛状态 (GameStatus, 0x0001) ──────────────────────────────────────────
uint8  game_progress         # 当前比赛阶段 (0-4)
uint16 stage_remain_time     # 当前阶段剩余时间 (s)

# ── 2. 机器人状态 (RobotStatus, 0x0201 / 0x0202) ─────────────────────────────
uint16 current_hp            # 本机当前血量
uint8  is_attacked           # 是否受到攻击 (0=否, 1=是)
uint16 shooter_heat          # 枪口热量

# ── 3. RFID 状态 (RFID, 0x0209 / 0x020A) ─────────────────────────────────────
bool   rfid_supply_arrived   # 补给区交互卡反馈
bool   rfid_control_arrived  # 控制区交互卡反馈

# ── 4. 机器人位姿 (RobotPosition, 0x0203) ────────────────────────────────────
float32 x                    # 当前位置 x (meter, map 坐标系)
float32 y                    # 当前位置 y (meter, map 坐标系)