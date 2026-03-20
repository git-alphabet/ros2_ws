#include "rm_behavior_tree/plugins/rmuc_2026/action/sub_enemy_mark.hpp"

namespace rm_behavior_tree
{

RmucSubEnemyMarkAction::RmucSubEnemyMarkAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUCEnemyMark>(name, conf, params)
{
}

BT::NodeStatus RmucSubEnemyMarkAction::onTick(
  const std::shared_ptr<rm_decision_interfaces::msg::RMUCEnemyMark> & last_msg)
{
  if (last_msg) {
    RCLCPP_DEBUG(
      logger(), "[%s] hero_vuln=%d engi_vuln=%d inf3_vuln=%d inf4_vuln=%d sentry_vuln=%d",
      name().c_str(), last_msg->enemy_hero_vuln, last_msg->enemy_engi_vuln,
      last_msg->enemy_infantry3_vuln, last_msg->enemy_infantry4_vuln,
      last_msg->enemy_sentry_vuln);
    setOutput("enemy_mark", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucSubEnemyMarkAction, "RmucSubEnemyMark");
