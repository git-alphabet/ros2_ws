#include "rm_behavior_tree/plugins/rmuc_2026/action/sub_team_hp.hpp"

namespace rm_behavior_tree
{

RmucSubTeamHPAction::RmucSubTeamHPAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUCTeamHP>(name, conf, params)
{
}

BT::NodeStatus RmucSubTeamHPAction::onTick(
  const std::shared_ptr<rm_decision_interfaces::msg::RMUCTeamHP> & last_msg)
{
  if (last_msg) {
    RCLCPP_DEBUG(
      logger(), "[%s] outpost=%d base=%d sentry=%d",
      name().c_str(), last_msg->outpost_hp, last_msg->base_hp, last_msg->sentry_hp);
    setOutput("team_hp", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucSubTeamHPAction, "RmucSubTeamHP");
