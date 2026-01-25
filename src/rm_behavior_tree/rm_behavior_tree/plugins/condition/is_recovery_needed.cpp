#include "rm_behavior_tree/plugins/condition/is_recovery_needed.hpp"

#include "behaviortree_cpp/exceptions.h"

namespace rm_behavior_tree
{

IsRecoveryNeededCondition::IsRecoveryNeededCondition(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::ConditionNode(name, conf), params_(params)
{
}

BT::NodeStatus IsRecoveryNeededCondition::tick()
{
  // 读黑板 need_recovery
  auto res = getInput<bool>("need_recovery");
  if (!res) {
    // 由于我们给了默认值 false，理论上不会走到这里；
    // 但保留防御性编程，便于定位配置错误。
    throw BT::RuntimeError("IsRecoveryNeeded: failed to read input [need_recovery]: ", res.error());
  }

  return (res.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsRecoveryNeededCondition, "IsRecoveryNeeded");
