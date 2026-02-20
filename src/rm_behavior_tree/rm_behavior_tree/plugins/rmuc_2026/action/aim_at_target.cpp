#include "rm_behavior_tree/plugins/rmuc_2026/action/aim_at_target.hpp"

namespace rm_behavior_tree
{

AimAtTargetAction::AimAtTargetAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf) {}

BT::NodeStatus AimAtTargetAction::onStart()
{
  std::string target;
  if (!getInput("target", target) || target.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  // TODO: 解析 "id:x:y" 并发布到云台控制话题
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AimAtTargetAction::onRunning()
{
  // TODO: 检查云台是否已对准目标
  // 简化实现：立即返回 SUCCESS（假设云台响应足够快）
  return BT::NodeStatus::SUCCESS;
}

void AimAtTargetAction::onHalted()
{
  // TODO: 停止云台追踪
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::AimAtTargetAction>("AimAtTarget");
}
