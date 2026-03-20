#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_SENTRY_DECISION_STATUS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_SENTRY_DECISION_STATUS_HPP_

#include <string>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/rmuc_sentry_decision_status.hpp"

namespace rm_behavior_tree
{
/// 订阅 0x020D 哨兵决策状态, 写入黑板 {sentry_decision_status}
class RmucSubSentryDecisionStatusAction
  : public BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUCSentryDecisionStatus>
{
public:
  RmucSubSentryDecisionStatusAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<rm_decision_interfaces::msg::RMUCSentryDecisionStatus>(
        "sentry_decision_status")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::RMUCSentryDecisionStatus> & last_msg)
    override;
};
}  // namespace rm_behavior_tree

#endif
