#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__AIM_AT_TARGET_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__AIM_AT_TARGET_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 接收 "id:x:y" 格式目标字符串，解析后向云台发送瞄准坐标
/// 实际瞄准由底层控制器执行，此节点仅设置黑板目标并返回 RUNNING
class AimAtTargetAction : public BT::StatefulActionNode
{
public:
  AimAtTargetAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target")};
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};
}  // namespace rm_behavior_tree
#endif
