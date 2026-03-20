#include "rm_behavior_tree/plugins/rmuc_2026/action/sub_projectile_allowance.hpp"

namespace rm_behavior_tree
{

RmucSubProjectileAllowanceAction::RmucSubProjectileAllowanceAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUCProjectileAllowance>(name, conf, params)
{
}

BT::NodeStatus RmucSubProjectileAllowanceAction::onTick(
  const std::shared_ptr<rm_decision_interfaces::msg::RMUCProjectileAllowance> & last_msg)
{
  if (last_msg) {
    RCLCPP_DEBUG(
      logger(), "[%s] ammo17=%d coins=%d fortress=%d",
      name().c_str(), last_msg->ammo_17mm, last_msg->remaining_coins,
      last_msg->fortress_ammo);
    setOutput("projectile_allowance", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(
  rm_behavior_tree::RmucSubProjectileAllowanceAction, "RmucSubProjectileAllowance");
