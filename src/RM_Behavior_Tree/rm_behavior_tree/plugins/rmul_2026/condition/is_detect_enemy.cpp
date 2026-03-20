#include "rm_behavior_tree/plugins/rmul_2026/condition/is_detect_enemy.hpp"

namespace rm_behavior_tree
{

IsDetectEnemyCondition::IsDetectEnemyCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsDetectEnemyCondition::checkEnemyDetected, this), config)
{
}

BT::NodeStatus IsDetectEnemyCondition::checkEnemyDetected()
{
  auto msg = getInput<bool>("message");

  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  return msg.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsDetectEnemyCondition>("IsDetectEnemy");
}
