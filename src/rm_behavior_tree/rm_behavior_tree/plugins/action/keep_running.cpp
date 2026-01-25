#include "rm_behavior_tree/plugins/action/keep_running.hpp"

namespace rm_behavior_tree
{

KeepRunningAction::KeepRunningAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus KeepRunningAction::tick()
{
  return BT::NodeStatus::RUNNING;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::KeepRunningAction>("KeepRunning");
}
