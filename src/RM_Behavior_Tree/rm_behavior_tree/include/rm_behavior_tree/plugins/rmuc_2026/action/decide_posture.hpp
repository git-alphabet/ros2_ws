#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DECIDE_POSTURE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DECIDE_POSTURE_HPP_

#include <string>
#include <cstdint>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/**
 * P4 姿态决策器 — 任务绑定 × 滞回稳定 × 衰减规避 × 增益感知
 *
 * 姿态编码: 1=进攻  2=防御  3=移动
 *
 * ── 优化建议 1: 任务绑定 ──
 *   当前活跃子树 → 默认姿态映射:
 *     BaseDefense / CriticalSurvival → 防御(2)
 *     EngageCombat / CombatLoop      → 进攻(1)
 *     ObjectivePlanner / PatrolAndScan / RespawnRecovery / WeaknessRecovery → 移动(3)
 *   通过 active_subtree (string) 输入实现。
 *
 * ── 优化建议 2: 滞回稳定 ──
 *   - 5s 硬冷却 (规则): 切换后 5s 内不允许再切
 *   - 收益阈值: 新姿态评分必须超过当前姿态 ≥15 分才触发切换
 *   - 任务优先级压制: 任务默认姿态权重最高 (+40)
 *
 * ── 补充 1: 衰减规避 ──
 *   单姿态累计 180s → 收益衰减 (进攻冷却-33%, 防御-50%, 移动功率-60%)
 *   在 170s 时主动切出再切回, 重置累计计时
 *   通过内部 posture_since_ms_ 追踪单姿态累计时长
 *
 * ── 补充 2: 增益感知 ──
 *   读取 0x0204 实时增益值 (buff_cool_value, buff_defense_pct, buff_vulnerability_pct)
 *   堡垒+进攻姿态冷却极高(315/s) → 在堡垒时强烈偏好进攻
 *
 * ── 补充 3: 0x0204 增益字段直接利用 ──
 *   实时冷却增益 → 调整进攻姿态评分
 *   实时防御增益 → 调整防御姿态评分
 *   实时易伤%   → 已在易伤时降低进攻偏好 (叠加易伤太危险)
 *
 * ── 补充 4: 裁判系统真实姿态反馈 ──
 *   读取 current_posture (0x020D bit12-13) 与内部维护对比
 *   不同步时以裁判系统为准, 修正内部状态
 */
class DecidePostureAction : public BT::SyncActionNode
{
public:
  DecidePostureAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      // ── 基础状态 ──
      BT::InputPort<int>("hp_cur"),
      BT::InputPort<int>("hp_max"),
      BT::InputPort<int>("heat_cur"),
      BT::InputPort<int>("heat_high"),
      BT::InputPort<bool>("has_target"),
      BT::InputPort<bool>("base_threat"),
      BT::InputPort<bool>("is_disengaged"),
      BT::InputPort<int>("stage_elapsed_time"),
      BT::InputPort<std::uint64_t>("now_ms"),
      // ── P4 新增: 任务绑定 (建议1) ──
      BT::InputPort<std::string>("active_subtree", "", "当前活跃子树名称"),
      // ── P4 新增: 裁判系统真实姿态 (补充4) ──
      BT::InputPort<int>("current_posture", "0", "裁判系统反馈的当前姿态 1/2/3, 0=未知"),
      // ── P4 新增: 实时增益 (补充2,3) ──
      BT::InputPort<int>("buff_cool_value", "0", "0x0204 冷却增益值"),
      BT::InputPort<int>("buff_defense_pct", "0", "0x0204 防御增益%"),
      BT::InputPort<int>("buff_vulnerability_pct", "0", "0x0204 易伤%"),
      // ── P4 新增: 弹量 ──
      BT::InputPort<int>("ammo_allow", "300", "当前允许发弹量"),
      // output
      BT::OutputPort<int>("posture_out")};
  }
  BT::NodeStatus tick() override;

private:
  // ── 滞回状态 (建议2) ──
  int last_posture_{3};              // 上次输出姿态 (初始=移动)
  std::uint64_t last_switch_ms_{0};  // 上次切换时刻 (5s 硬冷却)

  // ── 衰减追踪 (补充1) ──
  std::uint64_t posture_since_ms_{0}; // 当前姿态开始累计的时刻
  int tracked_posture_{3};            // 正在追踪的姿态

  static constexpr std::uint64_t SWITCH_COOLDOWN_MS = 5000;   // 5s 硬冷却
  static constexpr std::uint64_t DEGRADE_WARN_MS   = 170000;  // 170s 预衰减警告
  static constexpr std::uint64_t DEGRADE_MS        = 180000;  // 180s 衰减触发
  static constexpr int HYSTERESIS_THRESHOLD = 15;              // 收益阈值
};
}  // namespace rm_behavior_tree
#endif
