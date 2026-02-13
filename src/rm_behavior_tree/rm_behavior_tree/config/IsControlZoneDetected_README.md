# IsControlZoneDetected 插件使用说明

## 插件概述

`IsControlZoneDetected` 是一个 BehaviorTree Condition 节点，用于判断机器人是否成功与控制区产生交互。

## 功能说明

- **节点类型**: Condition（条件节点）
- **返回值**:
  - `SUCCESS`: 成功与控制区产生交互（`rfid_control_arrived == true`）
  - `FAILURE`: 未与控制区产生交互或数据不可用

## 输入端口

| 端口名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `rfid_status` | `rm_decision_interfaces::msg::RMUL` | `{rfid_status}` | RFID状态消息，包含控制区交互反馈 |

## 使用方法

### 在 XML 中使用

```xml
<!-- 检查是否在控制区内 -->
<IsControlZoneDetected rfid_status="{rfid_status}"/>
```

### 配合其他节点使用示例

```xml
<!-- 示例1: 到达控制区后执行某些操作 -->
<Sequence>
    <SendGoal goal_x="5.13" goal_y="-3.94" frame_id="map"/>
    <IsControlZoneDetected rfid_status="{rfid_status}"/>
    <RobotControl chassis_spin="True"/>
</Sequence>

<!-- 示例2: 循环等待直到进入控制区 -->
<ReactiveSequence>
    <Inverter>
        <IsControlZoneDetected rfid_status="{rfid_status}"/>
    </Inverter>
    <SendGoal goal_x="5.13" goal_y="-3.94" frame_id="map"/>
    <KeepRunning/>
</ReactiveSequence>
```

## 数据来源

该节点从 `RMUL.msg` 消息中读取 `rfid_control_arrived` 字段：

```
# RMUL.msg 中的相关字段
bool   rfid_control_arrived  # 控制区交互卡反馈
```

该数据通常由以下节点订阅并写入黑板：
- `SubRFIDStatus` - 订阅 `/rfid_status` 话题

## 典型应用场景

1. **控制区占领确认**: 导航到控制区后，确认是否成功占领
2. **条件触发**: 仅在占领控制区后执行特定战术动作
3. **状态监控**: 实时监控机器人是否在控制区范围内

## 注意事项

1. 确保 `SubRFIDStatus` 节点正常运行并订阅正确的话题
2. 如果黑板中没有 `rfid_status` 数据，节点会返回 `FAILURE`
3. 该节点不阻塞行为树执行，立即返回结果

## 相关节点

- `IsSupplyCardDetected` - 判断是否与补给区产生交互
- `IsWithinScope` - 判断机器人是否在指定范围内
- `SubRFIDStatus` - 订阅RFID状态数据

## 文件位置

- **头文件**: `include/rm_behavior_tree/plugins/condition/is_control_zone_detected.hpp`
- **源文件**: `plugins/condition/is_control_zone_detected.cpp`
- **注册名称**: `IsControlZoneDetected`
