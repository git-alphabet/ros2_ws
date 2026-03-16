#include "rm_behavior_tree/plugins/rmuc_2026/action/sub_sentry_decision_status.hpp"

namespace rm_behavior_tree
{

RmucSubSentryDecisionStatusAction::RmucSubSentryDecisionStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUCSentryDecisionStatus>(name, conf, params)
{
}

BT::NodeStatus RmucSubSentryDecisionStatusAction::onTick(
  const std::shared_ptr<rm_decision_interfaces::msg::RMUCSentryDecisionStatus> & last_msg)
{
  if (last_msg) {
    RCLCPP_DEBUG(
      logger(), "[%s] posture=%d can_free=%d can_instant=%d cost=%d",
      name().c_str(), last_msg->current_posture,
      last_msg->can_free_respawn, last_msg->can_instant_respawn,
      last_msg->instant_respawn_cost);
    setOutput("sentry_decision_status", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(
  rm_behavior_tree::RmucSubSentryDecisionStatusAction, "RmucSubSentryDecisionStatus");
