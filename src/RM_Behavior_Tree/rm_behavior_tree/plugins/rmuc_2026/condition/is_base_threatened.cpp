#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_base_threatened.hpp"

namespace rm_behavior_tree
{

IsBaseThreatenedCondition::IsBaseThreatenedCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf) {}

BT::NodeStatus IsBaseThreatenedCondition::tick()
{
  bool threat = false;
  int hp = 5000, hp_max = 5000;
  bool outpost_alive = true;
  getInput("base_threat", threat);
  getInput("base_hp_cur", hp);
  getInput("base_hp_max", hp_max);
  getInput("outpost_alive", outpost_alive);

  // P3 补充4: 前哨站存活 → 基地无敌 → 仅极端情况才报威胁
  if (outpost_alive) {
    // 前哨站在时基地不可被攻击，仅当 threat==true 且血量极低(<30%) 才认为威胁
    // (threat 可能来自敌人靠近基地区域，需要结合血量确认)
    if (threat && hp_max > 0 && hp < hp_max * 3 / 10) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  // 前哨站被击毁后，正常灵敏度
  if (threat) return BT::NodeStatus::SUCCESS;
  if (hp_max > 0 && hp < hp_max / 2) return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsBaseThreatenedCondition>("IsBaseThreatened");
}
