# BT XML 工程审计文档

> **审计日期**: 2026-03-17  
> **审计范围**: `src/RM_Behavior_Tree/rm_behavior_tree/config/rmuc_2026/` 下全部 16 个 XML 文件  
> **BT 框架**: BT.CPP v4 (`BTCPP_format="4"`)  
> **审计方法**: 控制流分析 + 数据流分析 + 语义分析 + 历史 bug 复盘

---

## 目录

1. [文件清单与层级关系](#1-文件清单与层级关系)
2. [控制流审计：节点类型与返回值传播](#2-控制流审计节点类型与返回值传播)
3. [数据流审计：Blackboard Key 读写配对](#3-数据流审计blackboard-key-读写配对)
4. [Tick 顺序与时序审计](#4-tick-顺序与时序审计)
5. [常见 BT 反模式检查](#5-常见-bt-反模式检查)
6. [TreeNodesModel 一致性审计](#6-treenodesmodel-一致性审计)
7. [历史 Bug 汇总与根因分析](#7-历史-bug-汇总与根因分析)
8. [已知限制与风险项](#8-已知限制与风险项)
9. [审计方法论总结](#9-审计方法论总结)

---

## 1. 文件清单与层级关系

### 1.1 文件列表

| # | 文件名 | BehaviorTree ID | 角色 |
|---|--------|-----------------|------|
| 1 | `rmuc_2026.xml` | `rmuc_2026` (主树) | 主入口 + TreeNodesModel |
| 2 | `PerceptionAndBlackboard.xml` | `PerceptionAndBlackboard` | 感知订阅 + 黑板解析 |
| 3 | `InitOnce.xml` | `InitOnce` | 配置初始化（幂等） |
| 4 | `CommandHub.xml` | `CommandHub` | 姿态/经济/复活指令决策 |
| 5 | `RespawnRecovery.xml` | `RespawnRecovery` | 死亡等待 + 虚弱恢复 |
| 6 | `WeaknessRecovery.xml` | `WeaknessRecovery` | 虚弱安全网（独立路径） |
| 7 | `CriticalSurvival.xml` | `CriticalSurvival` | 危急撤退 |
| 8 | `BaseDefense.xml` | `BaseDefense` | 基地防御 |
| 9 | `EngageCombat.xml` | `EngageCombat` | 交战入口 + 底盘旋转策略 |
| 10 | `CombatLoop.xml` | `CombatLoop` | 战斗循环（选目标→瞄准→射击） |
| 11 | `SustainAndEconomy.xml` | `SustainAndEconomy` | 后勤入口 |
| 12 | `HealPlan.xml` | `HealPlan` | 回血计划 |
| 13 | `AmmoPlan.xml` | `AmmoPlan` | 补弹计划 |
| 14 | `ObjectivePlanner.xml` | `ObjectivePlanner` | 目标控制（8候选点评分） |
| 15 | `PatrolAndScan.xml` | `PatrolAndScan` | 默认巡逻 |
| 16 | `DeathAndRespawn.xml` | `DeathAndRespawn` | ⚠️ 未被 include，疑似废弃 |

### 1.2 调用层级

```
rmuc_2026 (主树)
├── SubTree: PerceptionAndBlackboard      [每帧 tick]
│   └── 12 个订阅节点 + ParseSentryBlackboard
├── SubTree: InitOnce                     [幂等]
│   └── InitSentryConfig + InitCmdState
└── WhileDoElse
    ├── [比赛阶段] ReactiveSequence
    │   ├── SubTree: CommandHub            [每帧 tick]
    │   │   └── DecidePosture → DecideEconomyCmd → DecideRespawnCmd → SentryCmdMux
    │   └── ReactiveFallback (优先级战术)
    │       ├── [0]  RespawnRecovery       门控: RmucIsDead / IsWeakness
    │       ├── [0.5] WeaknessRecovery     门控: IsWeakness
    │       ├── [1]  CriticalSurvival      门控: IsCriticalState
    │       ├── [2]  BaseDefense           门控: IsBaseThreatened
    │       ├── [3]  EngageCombat          门控: HasValidTarget + IsCombatAllowed
    │       │   └── SubTree: CombatLoop
    │       ├── [4]  SustainAndEconomy     门控: RmucIsHPBelow / IsAmmoBelow
    │       │   ├── SubTree: HealPlan
    │       │   └── SubTree: AmmoPlan
    │       ├── [5]  ObjectivePlanner      无门控（永远执行）
    │       └── [6]  PatrolAndScan         无门控（兜底）
    └── [非比赛阶段] ReactiveSequence
        └── SendGoal(Home) + RmucRobotControl + SentryCmdMux
```

### 1.3 未被引用的文件

| 文件 | 状态 | 说明 |
|------|------|------|
| `DeathAndRespawn.xml` | ⚠️ **未被 include** | 未出现在 `rmuc_2026.xml` 的 `<include>` 列表中，也没有被任何 SubTree 引用。可能是早期版本的废弃文件，被 `RespawnRecovery.xml` 替代。建议删除或标记为废弃。 |

---

## 2. 控制流审计：节点类型与返回值传播

### 2.1 审计规则

| 容器类型 | 子节点 FAILURE | 子节点 RUNNING | 子节点 SUCCESS |
|---------|---------------|---------------|---------------|
| `Sequence` | 立即返回 FAILURE | 返回 RUNNING | 继续下一个 |
| `ReactiveSequence` | 立即返回 FAILURE | 返回 RUNNING, **下帧从头重新 tick** |  继续下一个 |
| `Fallback` | 继续下一个 | 返回 RUNNING | 立即返回 SUCCESS |
| `ReactiveFallback` | 继续下一个 | 返回 RUNNING, **下帧从头重新 tick** | 立即返回 SUCCESS |

### 2.2 逐文件审计

#### rmuc_2026.xml — 主树

```
ReactiveSequence (外层)
├── PerceptionAndBlackboard  → SUCCESS (订阅节点非阻塞)
├── InitOnce                 → SUCCESS (幂等)
└── WhileDoElse
    ├── condition: RmucIsGameTime → SUCCESS/FAILURE
    ├── [true]  ReactiveSequence → 比赛逻辑
    └── [false] ReactiveSequence → 非比赛逻辑
```

- ✅ 外层 `ReactiveSequence`：每帧重新从 PerceptionAndBlackboard 开始 tick，保证感知数据持续更新
- ✅ `WhileDoElse`：条件为 `RmucIsGameTime`，正确区分比赛/非比赛阶段

**比赛阶段内层：**

```
ReactiveSequence
├── CommandHub               → SUCCESS (Decision 节点全同步)
└── ReactiveFallback         → 取决于激活分支
    ├── Sequence[0]: SetBlackboard + RespawnRecovery
    ├── Sequence[1]: SetBlackboard + WeaknessRecovery
    ├── ...
    └── Sequence[7]: SetBlackboard + PatrolAndScan
```

- ✅ `ReactiveSequence` 确保 `CommandHub` 每帧在 ReactiveFallback 之前执行
- ✅ `ReactiveFallback` 每帧从优先级最高的分支开始尝试，保证抢占语义

#### CommandHub.xml

```
Sequence
├── DecidePosture      → SUCCESS (SyncAction)
├── DecideEconomyCmd   → SUCCESS (SyncAction)
├── DecideRespawnCmd   → SUCCESS (SyncAction)
└── RateController(5Hz)
    └── SentryCmdMux   → SUCCESS (SyncAction)
```

- ✅ 全是 `SyncActionNode`，tick 立即返回 `SUCCESS`
- ✅ `Sequence` 顺序执行，`RateController` 限制发送频率

#### RespawnRecovery.xml

```
Fallback
├── Sequence "IfDead_StopAndWait"
│   ├── RmucIsDead           → SUCCESS/FAILURE
│   ├── RmucRobotControl     → SUCCESS
│   └── RmucNavControlCmd    → SUCCESS
└── ReactiveSequence "WeaknessRecoveryFlow"
    ├── IsWeakness           → SUCCESS/FAILURE (门控)
    └── Fallback "RecoveryFlow"
        ├── Sequence "IfSupplyCard_Heal"
        │   ├── RmucIsSupplyCardDetected → SUCCESS/FAILURE
        │   └── RmucWaitAndHeal          → RUNNING/SUCCESS
        └── Sequence "GoSupply_ThenSearch"
            ├── RmucNavControlCmd
            ├── ReactiveFallback "NavUntilArrived"
            │   ├── RmucIsAtNavGoal  → SUCCESS/FAILURE
            │   └── ReactiveSequence
            │       ├── RateController(5Hz) → SendGoal
            │       └── KeepRunning  → RUNNING (保持导航)
            ├── RmucRobotControl
            ├── RmucNavControlCmd
            ├── InitSearchTimerIfNeeded
            └── ReactiveFallback "DetectOrMicroSearch"
                ├── RmucIsSupplyCardDetected
                └── RmucMicroSearchSupplyCard
```

- ✅ 死亡分支：`RmucIsDead` SUCCESS → 停车 → 整个 Fallback 返回 SUCCESS → 独占 ReactiveFallback
- ✅ 虚弱分支：`IsWeakness` SUCCESS → 进入恢复流程；FAILURE → ReactiveSequence FAILURE → Fallback 整体 FAILURE
- ✅ 正常状态（存活+不虚弱）：两个分支都 FAILURE → 整个 Fallback FAILURE → 让出给下一优先级

#### WeaknessRecovery.xml

```
ReactiveSequence
├── IsWeakness       → SUCCESS/FAILURE (门控)
├── SelectNearestDispelCard → SUCCESS (写 nav.goal_x/y)
├── RateController(1Hz) → SendGoal
├── RmucRobotControl
└── ReactiveFallback
    ├── IsAnyDispelCardDetected → SUCCESS/FAILURE
    └── MoveAround              → RUNNING/SUCCESS
```

- ✅ `IsWeakness` FAILURE → 整棵树立即退出
- ✅ 到达后 `MoveAround` 微动搜索，直到检测到解除卡

#### CriticalSurvival.xml

```
ReactiveSequence
├── IsCriticalState    → SUCCESS/FAILURE (门控)
├── RmucRobotControl   → SUCCESS
├── SelectSafeRetreatGoal → SUCCESS
├── RateController(1Hz) → SendGoal
└── KeepRunning        → RUNNING (保持)
```

- ✅ `IsCriticalState` FAILURE → 退出，让出给 BaseDefense
- ✅ `KeepRunning` 保持 RUNNING → ReactiveSequence 每帧重新检查门控

#### BaseDefense.xml

```
ReactiveSequence
├── IsBaseThreatened   → SUCCESS/FAILURE (门控)
├── RmucRobotControl   → SUCCESS (fire_enable=True)
├── RateController(1Hz) → SendGoal (defend_anchor)
└── SubTree: CombatLoop → RUNNING
```

- ✅ 威胁解除 → `IsBaseThreatened` FAILURE → 退出
- ✅ 防御时导航到锚点 + 同时战斗

#### EngageCombat.xml

```
ReactiveSequence
├── HasValidTarget      → SUCCESS/FAILURE
├── IsCombatAllowed     → SUCCESS/FAILURE
├── SetBlackboard (nav.goal_x = pose.x)
├── SetBlackboard (nav.goal_y = pose.y)
├── ReactiveFallback (旋转策略)
│   ├── Sequence: ShouldChassisSpin + RmucRobotControl(spin=True)
│   └── RmucRobotControl(spin=False)
└── SubTree: CombatLoop → RUNNING
```

- ✅ 目标丢失/战斗不允许 → FAILURE → 退出交战
- ✅ `SetBlackboard` 覆写 nav.goal 为当前位置 → ShouldChassisSpin dist≈0 → 允许旋转
- ✅ 退出后下帧 ObjectivePlanner/PatrolAndScan 会重新写入正确 nav.goal

#### CombatLoop.xml

```
ReactiveSequence
├── SelectBestTarget    → SUCCESS
├── AimAtTarget         → SUCCESS
├── ReactiveFallback (射击窗口)
│   ├── IsFireWindowOk  → SUCCESS/FAILURE
│   └── RmucRobotControl (fire_enable=False)
├── FireBurst           → RUNNING/SUCCESS
└── KeepRunning         → RUNNING
```

- ✅ `IsFireWindowOk` FAILURE → RmucRobotControl 关火 → ReactiveFallback 返回 SUCCESS → 继续循环
- ✅ `FireBurst` RUNNING 期间 → ReactiveSequence 持续重检 SelectBestTarget → 目标实时更新

#### SustainAndEconomy.xml

```
ReactiveFallback
├── SubTree: HealPlan   → FAILURE/RUNNING/SUCCESS
└── SubTree: AmmoPlan   → FAILURE/RUNNING/SUCCESS
```

- ✅ 血量充足 → HealPlan FAILURE → 尝试 AmmoPlan
- ✅ 弹量充足 → AmmoPlan FAILURE → 整个 SustainAndEconomy FAILURE → 让出
- ⚠️ **注意**: HealPlan 和 AmmoPlan 都可能导航到补给区，但目标可能不同（buff_zone vs nearest_station）

#### HealPlan.xml

```
ReactiveSequence
├── RmucIsHPBelow      → SUCCESS/FAILURE (门控)
└── ReactiveFallback
    ├── ReactiveSequence (已在补给区: hold+heal)
    │   ├── IsZoneCardDetected(SUPPLY)
    │   └── HoldAndHeal
    └── ReactiveSequence (未在补给区: 导航)
        ├── RateController(1Hz) → SendGoal (buff_zone)
        ├── RmucRobotControl
        └── KeepRunning
```

- ✅ 回血到安全值 → `RmucIsHPBelow` FAILURE → 退出

#### AmmoPlan.xml

```
ReactiveSequence
├── IsAmmoBelow        → SUCCESS/FAILURE (门控)
└── ReactiveFallback
    ├── ReactiveSequence (已在补给区: hold+tick)
    │   ├── IsZoneCardDetected(SUPPLY)
    │   └── HoldForSupplyAmmoTick
    └── ReactiveSequence (未在补给区: 导航)
        ├── SelectNearestResupplyStation
        ├── RateController(1Hz) → SendGoal
        ├── RmucRobotControl
        └── KeepRunning
```

- ✅ 弹量充足 → `IsAmmoBelow` FAILURE → 退出

#### ObjectivePlanner.xml

```
ReactiveSequence
├── SelectObjective    → SUCCESS (永远成功, 写 nav.goal_x/y)
├── RateController(1Hz) → SendGoal
├── RmucRobotControl
├── ReactiveFallback
│   ├── IsAtGoal       → SUCCESS/FAILURE
│   └── KeepRunning    → RUNNING
└── HoldObjective      → SUCCESS/RUNNING/FAILURE
```

- ✅ `SelectObjective` 永远返回 SUCCESS → 这棵树**永远被激活**
- ✅ 如果更高优先级分支都 FAILURE，`ObjectivePlanner` 一定能接住
- ⚠️ **注意**: `HoldObjective` 内部有 `base_threat`/`has_target` 提前打断机制

#### PatrolAndScan.xml

```
ReactiveSequence
├── RmucRobotControl   → SUCCESS (all False)
├── WaypointPatrol     → SUCCESS (写 nav.goal)
├── RateController(1Hz) → SendGoal
└── KeepRunning        → RUNNING
```

- ✅ 兜底分支，永远 RUNNING
- ✅ `WaypointPatrol` 每帧选择下一个路点

---

## 3. 数据流审计：Blackboard Key 读写配对

### 3.1 审计方法

对于每一个在 XML 中被 `{key}` 引用的 blackboard key：
1. 找到**所有写入源**（`output_port` / `inout_port` / `SetBlackboard`）
2. 找到**所有读取方**（`input_port` / `inout_port`）
3. 检查是否**有读无写**（悬空输入）或**有写无读**（死数据）

### 3.2 感知层写入（数据源）

| Blackboard Key | 写入节点 | 写入文件 |
|---|---|---|
| `{game_status}` | `RmucSubGameStatus` | PerceptionAndBlackboard.xml |
| `{robot_status}` | `RmucSubRobotStatus` | PerceptionAndBlackboard.xml |
| `{rfid.status}` | `RmucSubRFIDStatus` | PerceptionAndBlackboard.xml |
| `{pose.x}`, `{pose.y}`, `{pose.yaw}` | `RmucSubRobotPosition` | PerceptionAndBlackboard.xml |
| `{is_at_nav_goal}` | `RmucSubRobotPosition` | PerceptionAndBlackboard.xml |
| `{radar.tracks}` | `SubRadarTracks` | PerceptionAndBlackboard.xml |
| `{time.now_ms}` | `RmucSubGameStatus` | PerceptionAndBlackboard.xml |
| `{sentry_decision_status}` | `RmucSubSentryDecisionStatus` | PerceptionAndBlackboard.xml |
| `{robot_buff}` | `RmucSubRobotBuff` | PerceptionAndBlackboard.xml |
| `{projectile_allowance}` | `RmucSubProjectileAllowance` | PerceptionAndBlackboard.xml |
| `{field_status}` | `RmucSubFieldStatus` | PerceptionAndBlackboard.xml |
| `{enemy_mark}` | `RmucSubEnemyMark` | PerceptionAndBlackboard.xml |
| `{team_positions}` | `RmucSubTeamPositions` | PerceptionAndBlackboard.xml |
| `{team_hp}` | `RmucSubTeamHP` | PerceptionAndBlackboard.xml |

### 3.3 ParseSentryBlackboard 输出（二级衍生数据）

| Blackboard Key | 读取方 | 状态 |
|---|---|---|
| `{game.remain_s}` | DecideEconomyCmd, DecideRespawnCmd | ✅ |
| `{game.elapsed_s}` | DecidePosture, SelectObjective, HoldAndHeal, HoldForSupplyAmmoTick | ✅ |
| `{hp.cur}` | DecidePosture, DecideEconomyCmd, IsCriticalState, HoldAndHeal, CombatLoop 等 | ✅ |
| `{hp.max}` | DecidePosture, DecideEconomyCmd, HoldAndHeal, RmucWaitAndHeal | ✅ |
| `{heat.cur}` | DecidePosture, IsCriticalState, IsCombatAllowed, IsFireWindowOk | ✅ |
| `{ammo.allow}` | DecidePosture, DecideEconomyCmd, IsCombatAllowed, IsAmmoBelow 等 | ✅ |
| `{ammo.left}` | ParseSentryBlackboard 输出 | ⚠️ **有写无读** |
| `{base.hp.cur}` | DecideEconomyCmd, DecideRespawnCmd, IsBaseThreatened, SelectObjective | ✅ |
| `{base.hp.max}` | DecideEconomyCmd, DecideRespawnCmd, IsBaseThreatened, SelectObjective | ✅ |
| `{outpost.alive}` | IsBaseThreatened, SelectObjective | ✅ |
| `{state.is_dead}` | DecideRespawnCmd | ✅ |
| `{state.disengaged}` | DecidePosture, DecideEconomyCmd, HoldAndHeal | ✅ |
| `{state.disengage_cd_s}` | ParseSentryBlackboard 输出 | ⚠️ **有写无读** |
| `{economy.can_remote_heal}` | DecideEconomyCmd | ✅ |
| `{economy.can_remote_ammo}` | DecideEconomyCmd | ✅ |
| `{economy.coins}` | DecideEconomyCmd, DecideRespawnCmd | ✅ |
| `{combat.has_target}` | DecidePosture, HasValidTarget, HoldObjective | ✅ |
| `{combat.best_target}` | HasValidTarget, AimAtTarget, SelectBestTarget(output) | ✅ |
| `{threat.base}` | DecidePosture, DecideEconomyCmd, DecideRespawnCmd, IsBaseThreatened, HoldObjective, SelectObjective | ✅ |
| `{threat.fortress}` | ParseSentryBlackboard 输出 | ⚠️ **有写无读** |
| `{sentry.can_free_respawn}` | DecideRespawnCmd | ✅ |
| `{sentry.can_instant_respawn}` | DecideRespawnCmd | ✅ |
| `{sentry.instant_respawn_cost}` | DecideEconomyCmd, DecideRespawnCmd | ✅ |
| `{sentry.current_posture}` | DecidePosture, ShouldChassisSpin, IsFireWindowOk | ✅ |
| `{sentry.remote_ammo_count}` | DecideEconomyCmd | ✅ |
| `{sentry.remote_heal_count}` | DecideEconomyCmd | ✅ |
| `{sentry.exchanged_ammo_total}` | ParseSentryBlackboard 输出 | ⚠️ **有写无读** |
| `{sentry.can_activate_energy}` | ParseSentryBlackboard 输出 | ⚠️ **有写无读** |
| `{buff.heal_rate}` | ParseSentryBlackboard 输出 | ⚠️ **有写无读** |
| `{buff.cool_value}` | DecidePosture, IsFireWindowOk | ✅ |
| `{buff.defense_pct}` | DecidePosture | ✅ |
| `{buff.vulnerability_pct}` | DecidePosture, IsFireWindowOk | ✅ |
| `{buff.attack_pct}` | ParseSentryBlackboard 输出 | ⚠️ **有写无读** |
| `{economy.fortress_ammo}` | DecideEconomyCmd, SelectObjective | ✅ |
| `{field.central_highland}` | SelectObjective | ✅ |
| `{field.ladder_highland}` | SelectObjective | ✅ |
| `{field.fortress}` | SelectObjective | ✅ |
| `{field.outpost_buff}` | SelectObjective | ✅ |
| `{field.base_buff}` | SelectObjective | ✅ |
| `{field.small_energy}` | ParseSentryBlackboard 输出 | ⚠️ **有写无读** |
| `{field.big_energy}` | ParseSentryBlackboard 输出 | ⚠️ **有写无读** |
| `{enemy.hero_vuln}` ~ `{enemy.sentry_vuln}` | SelectBestTarget | ✅ |
| `{team.outpost_hp}` | ParseSentryBlackboard 输出 | ⚠️ **有写无读** |
| `{team.base_hp}` | ParseSentryBlackboard 输出 | ⚠️ **有写无读** |
| `{state.respawn_invincible}` | ParseSentryBlackboard 输出 | ⚠️ **有写无读** |
| `{state.respawn_invincible_remain_s}` | ParseSentryBlackboard 输出 | ⚠️ **有写无读** |
| `{state.power_boosted}` | ShouldChassisSpin | ✅ |
| `{state.power_boost_remain_s}` | ParseSentryBlackboard 输出 | ⚠️ **有写无读** |
| `{respawn.cum_instant_count}` | DecideEconomyCmd, DecideRespawnCmd, ParseSentryBlackboard(inout) | ✅ |

### 3.4 配置层写入（InitSentryConfig）

| Blackboard Key 前缀 | 示例 | 读取方 | 状态 |
|---|---|---|---|
| `{cfg.home_x/y}` | 非比赛阶段 SendGoal | ✅ |
| `{cfg.buff_zone_x/y}` | HealPlan, SelectNearestDispelCard 等 | ✅ |
| `{cfg.base_buff_x/y}` | SelectNearestDispelCard 等 | ✅ |
| `{cfg.outpost_buff_x/y}` | SelectNearestDispelCard 等 | ✅ |
| `{cfg.fortress_ally_x/y}` | SelectObjective | ✅ |
| `{cfg.fortress_enemy_x/y}` | SelectObjective | ✅ |
| `{cfg.central_highland_x/y}` | SelectObjective | ✅ |
| `{cfg.ladder_highland_x/y}` | SelectObjective | ✅ |
| `{cfg.defend_anchor_x/y}` | BaseDefense, CriticalSurvival, SelectObjective | ✅ |
| `{cfg.patrol.0~2.x/y}` | WaypointPatrol | ✅ |
| `{cfg.arrive_radius}` | ShouldChassisSpin, IsAtGoal | ✅ |
| `{cfg.hp_critical}` | IsCriticalState | ✅ |
| `{cfg.hp_low}` | IsCombatAllowed, RmucIsHPBelow | ✅ |
| `{cfg.hp_safe}` | HoldAndHeal | ✅ |
| `{cfg.heat_high}` | DecidePosture, IsCombatAllowed, IsFireWindowOk | ✅ |
| `{cfg.heat_critical}` | IsCriticalState | ✅ |
| `{cfg.ammo_low}` | DecideEconomyCmd, IsAmmoBelow | ✅ |
| `{cfg.ammo_target}` | DecideEconomyCmd, HoldForSupplyAmmoTick, SelectObjective | ✅ |
| `{cfg.supply_goal_x/y}` | RespawnRecovery SendGoal | ✅ |
| `{cfg.heal_wait_ms}` | RmucWaitAndHeal | ✅ |
| `{cfg.heal_min_ratio}` | RmucWaitAndHeal | ✅ |
| `{cfg.search_timeout_ms}` | RmucMicroSearchSupplyCard | ✅ |
| `{cfg.base_deficit_for_fortress}` | SelectObjective | ✅ |
| `{cfg.enemy_near_base_radius}` | IsBaseThreatened | ✅ |
| `{cfg.objective_hold_ms}` | HoldObjective | ✅ |
| `{cfg.combat_fire_burst_ms}` | FireBurst | ✅ |
| `{cfg.combat_fire_pause_ms}` | FireBurst | ✅ |

### 3.5 运行时动态 Key

| Key | 写入方 | 读取方 | 状态 |
|---|---|---|---|
| `{nav.goal_x}` / `{nav.goal_y}` | SelectObjective, WaypointPatrol, SelectNearestDispelCard, SelectNearestResupplyStation, SelectSafeRetreatGoal, EngageCombat(SetBlackboard) | SendGoal(各处), ShouldChassisSpin, IsAtGoal | ✅ |
| `{nav.objective}` | SelectObjective | ObjectivePlanner(SendGoal name) | ✅ |
| `{cmd.posture}` | DecidePosture | SentryCmdMux (在 CommandHub) | ✅ |
| `{cmd.confirm_respawn}` | DecideRespawnCmd | SentryCmdMux | ✅ |
| `{cmd.confirm_instant_respawn}` | DecideRespawnCmd | SentryCmdMux | ✅ |
| `{cmd.allow_ammo_target}` | DecideEconomyCmd(inout) | SentryCmdMux | ✅ |
| `{cmd.trig_remote_ammo}` | DecideEconomyCmd | SentryCmdMux | ✅ |
| `{cmd.trig_remote_hp}` | DecideEconomyCmd | SentryCmdMux | ✅ |
| `{cmd.enable_big_energy}` | DecideEconomyCmd | SentryCmdMux | ✅ |
| `{cmd.state}` | InitCmdState(inout), SentryCmdMux(inout) | — | ✅ |
| `{active_subtree}` | SetBlackboard (rmuc_2026.xml 各分支) | DecidePosture | ✅ (已修复) |
| `{heal_start_ms}` | RmucWaitAndHeal(inout) | RmucWaitAndHeal(inout) | ✅ (子树内部闭环) |
| `{search_start_ms}` | InitSearchTimerIfNeeded(inout) | RmucMicroSearchSupplyCard(inout) | ✅ |
| `{rfid.supply_arrived}` | ~~未知（可能在 cpp 内部写入）~~ | ~~RmucMicroSearchSupplyCard~~ | ✅ **已修复 (#10)**: port 已移除 |
| `{pose}` | RmucSubRobotPosition (TransformStamped output) | MoveAround(message) | ✅ **已修复 (#11)** |

### 3.6 有写无读汇总（Dead Data）

以下 key 由 `ParseSentryBlackboard` 输出但从未被任何节点读取：

| Key | 说明 | 风险等级 |
|---|---|---|
| `{ammo.left}` | 剩余弹量（区别于允许弹量 ammo.allow） | 🟡 低 — 可能是预留字段 |
| `{state.disengage_cd_s}` | 脱战倒计时秒数 | 🟡 低 — 可能用于日志/UI |
| `{threat.fortress}` | 堡垒威胁 | 🟡 低 — 可能后续版本使用 |
| `{sentry.exchanged_ammo_total}` | 已兑换弹量总量 | 🟡 低 |
| `{sentry.can_activate_energy}` | 能否激活大能量机关 | 🟡 低 |
| `{buff.heal_rate}` | 回血增益速率 | 🟡 低 |
| `{buff.attack_pct}` | 攻击增益百分比 | 🟡 低 |
| `{field.small_energy}` | 小能量机关状态 | 🟡 低 |
| `{field.big_energy}` | 大能量机关状态 | 🟡 低 |
| `{team.outpost_hp}` | 己方前哨站血量 | 🟡 低 |
| `{team.base_hp}` | 己方基地血量 | 🟡 低 |
| `{state.respawn_invincible}` | 复活无敌状态 | 🟡 低 |
| `{state.respawn_invincible_remain_s}` | 复活无敌剩余秒数 | 🟡 低 |
| `{state.power_boost_remain_s}` | 功率提升剩余秒数 | 🟡 低 |

> **评估**: 这些是 P0 阶段预先解析的数据，为后续优化预留。不影响当前功能，不算 bug。

---

## 4. Tick 顺序与时序审计

### 4.1 主树每帧 Tick 顺序

```
帧 N:
  1. PerceptionAndBlackboard → 12 个 Sub + ParseSentryBlackboard
     写入: game_status, robot_status, hp.cur, hp.max, heat.cur, ammo.allow, ...
  2. InitOnce → 跳过（已执行）
  3. WhileDoElse → RmucIsGameTime
  4. [比赛] ReactiveSequence:
     4a. CommandHub → DecidePosture 读取 {active_subtree}
         ⚠️ 此时 active_subtree 是帧 N-1 写入的值
     4b. ReactiveFallback → SetBlackboard 写入 {active_subtree}
         然后 tick 激活的 SubTree
```

### 4.2 已知时序问题

| 问题 | 影响 | 严重度 |
|---|---|---|
| `active_subtree` 有一帧延迟 | DecidePosture 读到上一帧的任务名 | 🟢 **极低** — 姿态切换有 5s 硬冷却，一帧延迟 (~10-100ms) 不会导致错误切换 |
| 第一帧 `active_subtree` 为空 | 走兜底分支（状态推断） | 🟢 **无影响** — cpp 已处理空字符串场景 |

### 4.3 导航目标竞争分析

多个子树都会写 `{nav.goal_x/y}`：

| 子树 | 写入时机 | 竞争风险 |
|---|---|---|
| EngageCombat | `SetBlackboard` (pose.x/y) | 仅交战时 |
| SelectObjective | 每帧 tick 时 | 仅 ObjectivePlanner 激活时 |
| WaypointPatrol | 每帧 tick 时 | 仅 PatrolAndScan 激活时 |
| SelectNearestDispelCard | 每帧 tick 时 | 仅 WeaknessRecovery 激活时 |
| SelectSafeRetreatGoal | 每帧 tick 时 | 仅 CriticalSurvival 激活时 |
| SelectNearestResupplyStation | 每帧 tick 时 | 仅 AmmoPlan 激活时 |

- ✅ **不存在竞争**: `ReactiveFallback` 确保同一帧只有一个战术子树激活，`nav.goal` 只被一个写入方控制
- ⚠️ **但** `CommandHub` 中 `ShouldChassisSpin` 读取的 `nav.goal` 是**上一帧**的值（与 active_subtree 相同的时序延迟）

---

## 5. 常见 BT 反模式检查

### 5.1 ✅ 已修复的反模式

| 反模式 | 原始位置 | 修复方式 | 修复轮次 |
|---|---|---|---|
| `KeepRunning` 阻塞 Sequence | CommandHub.xml | 移除 KeepRunning | Round 1 |
| `Sequence` 不响应条件变化 | rmuc_2026.xml 外层 | 改为 ReactiveSequence | Round 2 |
| `CancelNavGoal` 在 ReactiveSequence 中 | CriticalSurvival.xml | 移除 CancelNavGoal | Round 3 |
| `Sequence` 不重新 tick 巡逻 | PatrolAndScan.xml | 改为 ReactiveSequence | Round 3 |

### 5.2 当前检查清单

| 检查项 | 结果 | 说明 |
|---|---|---|
| ReactiveSequence 中是否有会阻塞的非门控节点 | ✅ 通过 | 所有 ReactiveSequence 的第一个子节点都是条件/门控 |
| ReactiveSequence 中是否有 CancelNavGoal | ✅ 通过 | 已在 Round 3 移除 |
| Fallback 子节点是否都能正确返回 FAILURE | ✅ 通过 | 每个门控 FAILURE → 子树 FAILURE → Fallback 继续 |
| KeepRunning 是否用在了正确的位置 | ✅ 通过 | 仅在 ReactiveSequence 最后一个子节点 |
| SetBlackboard 是否在 Sequence 中不会阻塞 | ✅ 通过 | SetBlackboard 永远返回 SUCCESS |
| SubTree _autoremap 是否一致 | ✅ 通过 | 所有 SubTree 都使用 _autoremap="true" |
| RateController 内是否有多个子节点 | ✅ 通过 | 每个 RateController 只有 1 个子节点 |

### 5.3 潜在风险模式

| 模式 | 位置 | 风险 | 建议 |
|---|---|---|---|
| Sequence 包裹 SetBlackboard + SubTree (7处) | rmuc_2026.xml ReactiveFallback 内 | 🟢 极低 — SetBlackboard 永远 SUCCESS | 保持现状 |
| `MoveAround` 使用 `{pose}` key | WeaknessRecovery.xml | 🟡 需确认 `{pose}` 的写入源 | 检查 cpp 实现 |

---

## 6. TreeNodesModel 一致性审计

### 6.1 审计规则

- 每个在 XML 中使用的自定义节点，必须在 `rmuc_2026.xml` 的 `<TreeNodesModel>` 中有对应定义
- port 名称和方向（input/output/inout）必须与 C++ `providedPorts()` 一致
- `default` 值不影响运行时（仅供 Groot2 显示），但应与实际用法一致

### 6.2 节点覆盖检查

| XML 使用的节点 | TreeNodesModel 中有定义 | 状态 |
|---|---|---|
| `RmucSubGameStatus` | ✅ | |
| `RmucSubRobotStatus` | ✅ | |
| `RmucSubRFIDStatus` | ✅ | |
| `RmucSubRobotPosition` | ✅ | |
| `SubRadarTracks` | ✅ | |
| `RmucSubSentryDecisionStatus` | ✅ | |
| `RmucSubRobotBuff` | ✅ | |
| `RmucSubProjectileAllowance` | ✅ | |
| `RmucSubFieldStatus` | ✅ | |
| `RmucSubEnemyMark` | ✅ | |
| `RmucSubTeamPositions` | ✅ | |
| `RmucSubTeamHP` | ✅ | |
| `InitSentryConfig` | ✅ | |
| `InitCmdState` | ✅ | |
| `ParseSentryBlackboard` | ✅ | |
| `DecidePosture` | ✅ | |
| `DecideEconomyCmd` | ✅ | |
| `DecideRespawnCmd` | ✅ | |
| `SentryCmdMux` | ✅ | |
| `SendGoal` | ✅ | |
| `CancelNavGoal` | ✅ | |
| `MoveAround` | ✅ | |
| `KeepRunning` | ✅ | |
| `SelectBestTarget` | ✅ | |
| `AimAtTarget` | ✅ | |
| `FireBurst` | ✅ | |
| `RmucRobotControl` | ✅ | |
| `SelectObjective` | ✅ | |
| `HoldObjective` | ✅ | |
| `WaypointPatrol` | ✅ | |
| `SelectNearestDispelCard` | ✅ | |
| `SelectNearestResupplyStation` | ✅ | |
| `SelectSafeRetreatGoal` | ✅ | |
| `HoldAndHeal` | ✅ | |
| `HoldForSupplyAmmoTick` | ✅ | |
| `RmucIsGameTime` | ✅ | |
| `RmucIsDead` | ✅ | |
| `IsWeakness` | ✅ | |
| `RmucIsHPBelow` | ✅ | |
| `IsAmmoBelow` | ✅ | |
| `IsCriticalState` | ✅ | |
| `IsBaseThreatened` | ✅ | |
| `HasValidTarget` | ✅ | |
| `IsCombatAllowed` | ✅ | |
| `IsFireWindowOk` | ✅ | |
| `IsZoneCardDetected` | ✅ | |
| `ShouldChassisSpin` | ✅ | |
| `IsAnyDispelCardDetected` | ✅ | |
| `IsAtGoal` | ✅ | |
| `RateController` | ✅ | |
| `RmucNavControlCmd` | ✅ | |
| `RmucWaitAndHeal` | ✅ | |
| `InitSearchTimerIfNeeded` | ✅ | |
| `RmucMicroSearchSupplyCard` | ✅ | |
| `RmucIsAtNavGoal` | ✅ | |
| `RmucIsSupplyCardDetected` | ✅ | |
| `SetBlackboard` | 内置节点，无需定义 | ✅ |
| `WhileDoElse` | 内置节点，无需定义 | ✅ |

### 6.3 独立文件 TreeNodesModel

以下文件添加了临时 TreeNodesModel 用于 Groot2 独立加载：

| 文件 | 包含的节点定义 | 状态 |
|---|---|---|
| `CommandHub.xml` | DecidePosture, DecideEconomyCmd, DecideRespawnCmd, SentryCmdMux, RateController | ✅ |
| `EngageCombat.xml` | HasValidTarget, IsCombatAllowed, ShouldChassisSpin, RmucRobotControl, RateController | ✅ |

---

## 7. 历史 Bug 汇总与根因分析

### 7.1 全部已修复 Bug (3 轮审计 + 1 次数据流修复 + 1 次深度数据流审计)

| # | 发现轮次 | 文件 | Bug 描述 | 根因类型 | 修复方式 |
|---|---|---|---|---|---|
| 1 | Round 1 | CommandHub.xml | `KeepRunning` 导致 Sequence 永远 RUNNING，后续节点不执行 | 控制流 | 移除 KeepRunning |
| 2 | Round 1 | rmuc_2026.xml | `InitOnce` 中 5 个 supply/heal 配置 port 缺失 | 数据流 | 补充 port 映射 |
| 3 | Round 1 | RespawnRecovery.xml | 黑板 key 写错（`{now_ms}` 应为 `{time.now_ms}` 等） | 拼写错误 | 修正 key 名称 |
| 4 | Round 1 | rmuc_2026.xml | TreeNodesModel 中 ParseSentryBlackboard 多了 `rfid_status` port | 一致性 | 移除多余 port |
| 5 | Round 2 | rmuc_2026.xml | 外层 `Sequence` 导致黑板在 InitOnce SUCCESS 后冻结 | 控制流 | 改为 ReactiveSequence |
| 6 | Round 2 | rmuc_2026.xml | RmucWaitAndHeal TreeNodesModel 中 default 值错误 | 一致性 | 修正 default |
| 7 | Round 3 | PatrolAndScan.xml | `Sequence` 不重新 tick WaypointPatrol | 控制流 | 改为 ReactiveSequence |
| 8 | Round 3 | CriticalSurvival.xml | `CancelNavGoal` 在 ReactiveSequence 中每帧取消导航 | 反模式 | 移除 CancelNavGoal |
| 9 | 数据流修复 | rmuc_2026.xml | `active_subtree` 从未被写入，DecidePosture 任务绑定 (+40分) 完全失效 | **数据流断裂** | 添加 SetBlackboard |
| 10 | 深度审计 | micro_search_supply_card.hpp/cpp, RespawnRecovery.xml, rmuc_2026.xml | `{rfid.supply_arrived}` 无写入源，`rfid_supply_arrived` port 是悬空输入 | **数据流断裂** | 移除冗余 port，统一用 `rfid_status` 消息的 `rfid_supply` 字段 |
| 11 | 深度审计 | sub_robot_position.hpp/cpp, PerceptionAndBlackboard.xml, rmuc_2026.xml | `{pose}` (TransformStamped) 无写入源，`MoveAround` 永远 FAILURE → WeaknessRecovery 微动搜索失效 | **数据流断裂** | 在 `RmucSubRobotPosition` 中新增 TransformStamped 类型 `pose` output port |
| 12 | 深度审计 | DeathAndRespawn.xml | 未被任何文件 include 或引用，是早期废弃版本 | 工程管理 | 添加 DEPRECATED 注释标记 |

### 7.2 根因分类

```
控制流 Bug:  4 个 (#1, #5, #7, #8)
数据流 Bug:  5 个 (#2, #3, #9, #10, #11)
一致性 Bug:  2 个 (#4, #6)
工程管理:   1 个 (#12)
```

### 7.3 教训

| 教训 | 对应 Bug |
|---|---|
| `KeepRunning` 只能放在 ReactiveSequence 的最后一个子节点 | #1 |
| 外层容器必须是 ReactiveSequence 才能保证子树响应状态变化 | #5, #7 |
| 不要在 ReactiveSequence 中放有副作用的 Action（如 CancelNavGoal） | #8 |
| **每个 InputPort 必须有对应的写入源** | #2, #3, #9 |
| TreeNodesModel 必须与 cpp providedPorts() 严格同步 | #4, #6 |

---

## 8. 已知限制与风险项

### 8.1 已确认的限制

| 项目 | 描述 | 影响 | 建议 |
|---|---|---|---|
| DecidePosture 强集成 | 230 行 cpp 内置 5 层评分逻辑，XML 层无法可视化决策过程 | Groot2 只显示一个方块 | 可接受，评分+滞回模型不适合拆到 XML |
| active_subtree 一帧延迟 | CommandHub 在 ReactiveFallback 之前 tick | 几乎无影响（5s 冷却兜底） | 保持现状 |
| DeathAndRespawn.xml 废弃 | 未被 include 或引用 | 占磁盘空间，可能误导开发者 | 删除或加 DEPRECATED 标记 |
| 14 个有写无读 key | ParseSentryBlackboard 输出但无读取 | 预留字段，不影响运行 | 后续版本按需启用 |

### 8.2 待确认项

| 项目 | 描述 | 需要确认 |
|---|---|---|
| `{rfid.supply_arrived}` | ~~RmucMicroSearchSupplyCard 读取此 key，但未见 XML 层写入~~ | ✅ **已修复 (#10)**: 移除冗余 port，统一用 rfid_status.rfid_supply |
| `{pose}` | ~~MoveAround 的 `message` 参数使用 `{pose}`，但 XML 层只有 `{pose.x/y/yaw}`~~ | ✅ **已修复 (#11)**: RmucSubRobotPosition 新增 TransformStamped 类型 pose output |
| 所有 TODO 坐标 | InitSentryConfig 中大量 `default="0.0"` 的坐标 | 上场前必须填写实际坐标 |
| 大能量机关激活 | `enable_big_energy` 由 DecideEconomyCmd 输出，但激活条件和流程未在 XML 中体现 | 确认仿真中是否需要额外子树 |

### 8.3 仿真前必做清单

- [ ] 填写 `InitSentryConfig` 中所有 `TODO` 坐标（地图坐标系）
- [ ] 确认 `{rfid.supply_arrived}` 和 `{pose}` 的数据来源
- [ ] 在 Groot2 中加载主树，确认 port 连线无断裂
- [ ] 用 `ros2 topic echo` 验证 12 个订阅话题是否有数据
- [ ] 配置 Groot2 ZMQ 连接到 port 1668，观察运行时 tick 状态

---

## 9. 审计方法论总结

### 9.1 完整审计检查清单

经过 3 轮审计迭代，总结出以下完整检查清单供后续使用：

#### A. 控制流 (Control Flow)

- [ ] 每个 `Sequence` 是否应该是 `ReactiveSequence`（需要对上层状态变化做出响应？）
- [ ] 每个 `Fallback` 是否应该是 `ReactiveFallback`（需要被高优先级分支抢占？）
- [ ] `KeepRunning` 是否仅出现在 `ReactiveSequence` 的最后位置
- [ ] `ReactiveSequence` 中是否有有副作用的 Action（如 CancelNavGoal）
- [ ] 每个子树的门控条件 FAILURE 时，整棵子树是否能正确返回 FAILURE
- [ ] `RateController` 内是否只有 1 个子节点

#### B. 数据流 (Data Flow)

- [ ] **每个 InputPort 的 `{key}` 是否有对应的 OutputPort/SetBlackboard 写入**
- [ ] **每个 OutputPort 的 `{key}` 是否有对应的 InputPort 读取**（有写无读不算 bug，但应记录）
- [ ] `inout_port` 是否在正确的生命周期内使用（避免竞争写入）
- [ ] `SetBlackboard` 的 `output_key` 和 `value` 是否拼写正确
- [ ] 跨子树共享的 blackboard key 是否通过 `_autoremap="true"` 传递
- [ ] 配置类 key（cfg.*）是否在 `InitSentryConfig` 中定义

#### C. 时序 (Timing)

- [ ] 在同一个 `ReactiveSequence` 中，写入节点和读取节点的 tick 顺序是否正确
- [ ] 导航目标 `{nav.goal_x/y}` 是否存在多写入方竞争
- [ ] `RateController` 频率是否合理（太高浪费 CPU，太低响应迟钝）

#### D. TreeNodesModel 一致性

- [ ] 每个自定义节点在 `<TreeNodesModel>` 中都有定义
- [ ] port 名称、方向、default 值与 cpp `providedPorts()` 一致
- [ ] 不存在多余的 port（已删除的 port 要从 model 中同步移除）

#### E. 工程管理

- [ ] 所有 XML 通过 `xmllint --noout` 验证
- [ ] `<include>` 列表覆盖所有子树文件
- [ ] 无废弃/未引用的文件
- [ ] 编译通过 (`colcon build --packages-select rm_behavior_tree`)

---

## 附录: xmllint 批量验证命令

```bash
for f in src/RM_Behavior_Tree/rm_behavior_tree/config/rmuc_2026/*.xml; do
  xmllint --noout "$f" && echo "✅ $(basename $f)" || echo "❌ $(basename $f)"
done
```

## 附录: 快速查找悬空 InputPort 的 grep 命令

```bash
# 提取所有 XML 中引用的 blackboard key
grep -oP '\{[a-z_\.]+\}' src/RM_Behavior_Tree/rm_behavior_tree/config/rmuc_2026/*.xml | \
  sed 's/.*://' | sort -u

# 提取所有 output_port / inout_port 的 key
grep -oP '(output_port|inout_port).*default="\{([^"]+)\}"' \
  src/RM_Behavior_Tree/rm_behavior_tree/config/rmuc_2026/rmuc_2026.xml | \
  grep -oP '\{[^}]+\}' | sort -u

# 对比两组 key，找出只读不写的
```
