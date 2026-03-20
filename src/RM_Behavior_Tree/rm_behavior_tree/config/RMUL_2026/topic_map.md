# RMUL_2026 行为树 话题订阅/发布汇总

## 📥 订阅的话题（Subscriber）

| 话题名 | 消息类型 | 插件 | 写入黑板变量 |
|--------|----------|------|--------------|
| `/robot_status` | `rm_decision_interfaces/msg/RMUL` | `SubRobotStatus` | `{robot_status}` |
| `/game_status` | `rm_decision_interfaces/msg/RMUL` | `SubGameStatus` | `{game_status}` |
| `/rfid_status` | `rm_decision_interfaces/msg/RMUL` | `SubRFIDStatus` | `{rfid.status}` |
| `/robot_position` | `rm_decision_interfaces/msg/RMUL` | `SubRobotPosition` | `{pose.x}`, `{pose.y}` |

> 以上 4 个订阅均在 `PerceptionAndBlackboard` 子树里每 tick 执行一次，将数据写入黑板，供全树共享读取。

---

## 📤 发布的话题（Publisher）

| 话题名 | 消息类型 | 插件 | 说明 |
|--------|----------|------|------|
| `robot_control` | `rm_decision_interfaces/msg/RMUL` | `RobotControl` | 控制云台扫描（`stop_gimbal_scan`）和底盘小陀螺（`chassis_spin`）。注册时 `default_port_value = "robot_control"` |
| `/nav_control_cmd` | `rm_decision_interfaces/msg/RMUL` | `NavControlCmd` | 控制导航模式（`cmd_type`）和紧急制动（`emergency_stop`）。注册时 `default_port_value = "/nav_control_cmd"` |

> `SendGoal` 通过 `navigate_to_pose` **Action Server** 发送导航目标，不是普通 topic pub，而是 ROS2 Action 调用。

---

## 🎯 黑板变量读取关系

| 黑板变量 | 写入插件 | 读取插件 |
|----------|----------|----------|
| `{robot_status}` | `SubRobotStatus` | `IsHPAbove` / `IsHPBelow` / `IsDetectEnemy` / `IsStatusOK` / `IsDead` / `IsFriendOK` |
| `{game_status}` | `SubGameStatus` | `IsGameTime` |
| `{rfid.status}` | `SubRFIDStatus` | `IsSupplyCardDetected` / `IsAtNavGoal` / `MicroSearchSupplyCard` |
| `{pose.x}` / `{pose.y}` | `SubRobotPosition` | `IsWithinScope` |

---

## 🔗 完整数据流链路

```
订阅链路:
  /robot_status   → SubRobotStatus   → {robot_status}
      └─ 读取方: IsHPAbove / IsHPBelow / IsDetectEnemy / IsStatusOK / IsDead / IsFriendOK

  /game_status    → SubGameStatus    → {game_status}
      └─ 读取方: IsGameTime

  /rfid_status    → SubRFIDStatus    → {rfid.status}
      └─ 读取方: IsSupplyCardDetected / IsAtNavGoal / MicroSearchSupplyCard

  /robot_position → SubRobotPosition → {pose.x} / {pose.y}
      └─ 读取方: IsWithinScope

发布链路:
  RobotControl   →  robot_control     (topic pub，控制云台/小陀螺)
  NavControlCmd  →  /nav_control_cmd  (topic pub，控制导航模式)
  SendGoal       →  navigate_to_pose  (ROS2 Action，发送导航目标点)
```

---

## ⚠️ 注意事项

- `RobotControl` 与 `NavControlCmd` 均使用同一消息类型 `rm_decision_interfaces/msg/RMUL`，但字段不同，发往不同话题名。
- 仿真环境引入了 namespace 前缀（如 `/red_standard_robot1`），实车不带 namespace，话题名以实际 launch 配置为准。
- `SubArmors`（订阅 `/detector/armors`）在 `TreeNodesModel` 中有声明，但当前 RMUL_2026 树的 `PerceptionAndBlackboard` 子树中**未使用**，`IsDetectEnemy` 实际读取的是 `{robot_status}` 而非 `{armors}`。
