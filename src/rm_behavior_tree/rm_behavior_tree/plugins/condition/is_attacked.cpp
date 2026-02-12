#include "rm_behavior_tree/plugins/condition/is_attacked.hpp"

namespace rm_behavior_tree
{

IsAttackedAction::IsAttackedAction(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsAttackedAction::checkRobotAttacked, this), config)
{
}

BT::NodeStatus IsAttackedAction::checkRobotAttacked()
{
  auto msg = getInput<std::shared_ptr<rm_decision_interfaces::msg::RMUL>>("message");

  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  if ((*msg)->is_attacked) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsAttackedAction>("IsAttacked");
}
