#include "rm_behavior_tree/plugins/action/sub_rfid_status.hpp"

#include "rclcpp/rclcpp.hpp"  // ✅ 确保 RCLCPP_DEBUG 可用

namespace rm_behavior_tree
{

SubRFIDStatusAction::SubRFIDStatusAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::RFID>(name, conf, params)
{
}

BT::NodeStatus SubRFIDStatusAction::onTick(
  const std::shared_ptr<rm_decision_interfaces::msg::RFID> & last_msg)
{
  if (last_msg)  // empty if no new message received, since the last tick
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, rfid_status: %s", name().c_str(),
      (last_msg->rfid_supply_arrived ? "true" : "false"));

    setOutput("rfid_status", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubRFIDStatusAction, "SubRFIDStatus");
