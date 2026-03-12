#include "rm_behavior_tree/plugins/rmul_2026/condition/is_at_nav_goal.hpp"

namespace rm_behavior_tree
{

IsAtNavGoal::IsAtNavGoal(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus IsAtNavGoal::tick()
{
  // 从黑板读取 bool（由 SubRobotPosition 写入 is_at_nav_goal 端口）
  auto res = getInput<bool>("nav_status");
  if (!res) {
    return BT::NodeStatus::FAILURE;
  }

  return res.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsAtNavGoal>("IsAtNavGoal");
}
