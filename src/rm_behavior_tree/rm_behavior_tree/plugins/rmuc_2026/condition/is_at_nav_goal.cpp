#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_at_nav_goal.hpp"

namespace rm_behavior_tree
{

RmucIsAtNavGoalCondition::RmucIsAtNavGoalCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus RmucIsAtNavGoalCondition::tick()
{
  // 从黑板读取 RMUC.msg
  rm_decision_interfaces::msg::RMUC rfid_msg;
  auto res = getInput<rm_decision_interfaces::msg::RMUC>("rfid_status");
  if (!res) {
    return BT::NodeStatus::FAILURE;
  }
  rfid_msg = res.value();

  // 检查是否到达导航目标点
  const bool is_at_goal = rfid_msg.is_at_nav_goal;

  return is_at_goal ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::RmucIsAtNavGoalCondition>("RmucIsAtNavGoal");
}
