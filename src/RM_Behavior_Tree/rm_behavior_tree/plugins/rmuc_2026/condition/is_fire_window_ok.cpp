#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_fire_window_ok.hpp"
#include <algorithm>

namespace rm_behavior_tree
{

IsFireWindowOkCondition::IsFireWindowOkCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf) {}

BT::NodeStatus IsFireWindowOkCondition::tick()
{
  int heat = 0, heat_high = 210, ammo = 300;
  int posture = 0, cool_val = 0, vuln_pct = 0, ammo_conserve = 30;
  getInput("heat_cur", heat);
  getInput("heat_high", heat_high);
  getInput("ammo_allow", ammo);
  getInput("current_posture", posture);
  getInput("buff_cool_value", cool_val);
  getInput("buff_vulnerability_pct", vuln_pct);
  getInput("ammo_conserve", ammo_conserve);

  // ═══════ 硬性门槛 (一票否决) ═══════

  // 1. 发射机构断电 → 无法射击
  auto msg_opt = getInput<std::shared_ptr<rm_decision_interfaces::msg::RMUCRobotStatus>>(
    "robot_status");
  if (msg_opt && msg_opt.value()) {
    const auto & r = *msg_opt.value();
    if (!r.shooter_power_output && r.current_hp > 0) {
      return BT::NodeStatus::FAILURE;
    }
  }

  // 2. 弹量 ≤ 0 → 无弹可打
  if (ammo <= 0) return BT::NodeStatus::FAILURE;

  // ═══════ 姿态感知的动态热量阈值 (优化建议 3 + 补充 2) ═══════
  //
  // 进攻姿态冷却 ×3, 堡垒再叠加 → 可以更激进地打
  // 防御姿态冷却 ×1/3 → 必须更保守
  //
  // effective_heat_high = heat_high * posture_factor
  //   进攻: × 1.0 (保持原阈值, 因为冷却快可以打到极限)
  //   防御: × 0.7 (冷却慢, 提前停火)
  //   移动: × 0.85 (冷却低, 略保守)
  //   未知: × 0.9
  double posture_factor = 0.9;
  if (posture == 1) {
    posture_factor = 1.0;
    // 有额外冷却增益 (堡垒/大能量机关) → 可以更激进
    if (cool_val > 30) posture_factor = 1.05;  // 允许超过 heat_high 5%
  } else if (posture == 2) {
    posture_factor = 0.70;
  } else if (posture == 3) {
    posture_factor = 0.85;
  }

  int effective_heat_high = static_cast<int>(heat_high * posture_factor);

  // 3. 热量 ≥ 动态阈值 → 停火
  if (heat >= effective_heat_high) return BT::NodeStatus::FAILURE;

  // ═══════ 软性判断: 弹量节约 ═══════
  //
  // 弹量 < ammo_conserve 时, 收紧热量阈值 → 减少浪费
  // (每少 10 发弹量, 热量阈值再降 5%)
  if (ammo < ammo_conserve && ammo_conserve > 0) {
    double conserve_factor = 0.95 - 0.05 * ((ammo_conserve - ammo) / 10);
    conserve_factor = std::max(0.6, conserve_factor);
    effective_heat_high = static_cast<int>(effective_heat_high * conserve_factor);
    if (heat >= effective_heat_high) return BT::NodeStatus::FAILURE;
  }

  // ═══════ 软性判断: 易伤状态 (补充 3) ═══════
  //
  // 处于易伤状态时, 受到的伤害增加 → 应更保守开火 (避免暴露位置)
  // 但不完全禁止 (有些情况必须打)
  // 策略: 易伤时热量阈值再降 15%
  if (vuln_pct > 0) {
    effective_heat_high = static_cast<int>(effective_heat_high * 0.85);
    if (heat >= effective_heat_high) return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsFireWindowOkCondition>("IsFireWindowOk");
}
