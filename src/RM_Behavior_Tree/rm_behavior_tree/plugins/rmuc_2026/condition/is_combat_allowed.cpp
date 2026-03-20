#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_combat_allowed.hpp"

namespace rm_behavior_tree
{

IsCombatAllowedCondition::IsCombatAllowedCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf) {}

BT::NodeStatus IsCombatAllowedCondition::tick()
{
  int ammo = 300, heat = 0, heat_high = 210, hp = 400, hp_low = 180;
  getInput("ammo_allow", ammo);
  getInput("heat_cur", heat);
  getInput("heat_high", heat_high);
  getInput("hp_cur", hp);
  getInput("hp_low", hp_low);

  // 检查发射机构是否有供电 (替代旧的 is_weak 字段)
  auto msg_opt = getInput<std::shared_ptr<rm_decision_interfaces::msg::RMUCRobotStatus>>(
    "robot_status");
  if (msg_opt && msg_opt.value()) {
    const auto & r = *msg_opt.value();
    // shooter 断电 + 存活 → 无法战斗
    if (!r.shooter_power_output && r.current_hp > 0) {
      return BT::NodeStatus::FAILURE;
    }
  }

  if (ammo <= 0) return BT::NodeStatus::FAILURE;
  if (heat >= heat_high) return BT::NodeStatus::FAILURE;
  if (hp < hp_low) return BT::NodeStatus::FAILURE;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsCombatAllowedCondition>("IsCombatAllowed");
}
