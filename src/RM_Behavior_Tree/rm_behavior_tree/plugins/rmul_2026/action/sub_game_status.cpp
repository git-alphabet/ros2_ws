#include "rm_behavior_tree/plugins/rmul_2026/action/sub_game_status.hpp"

namespace rm_behavior_tree
{

SubGameStatusAction::SubGameStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::RMULRob>(name, conf, params)
{
}

BT::NodeStatus SubGameStatusAction::onTick(
  const std::shared_ptr<rm_decision_interfaces::msg::RMULRob> & last_msg)
{
  // 每次 tick 都写入 ROS 时间（毫秒），供 IsNavTimeout 等节点使用
  const auto now_ns = node_->now().nanoseconds();
  setOutput("now_ms", static_cast<std::uint64_t>(now_ns / 1000000ULL));

  if (last_msg)  // empty if no new message received, since the last tick
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, game_progress: %s, remain: %s", name().c_str(),
      std::to_string(last_msg->game_progress).c_str(),
      std::to_string(last_msg->stage_remain_time).c_str());
    setOutput("game_status", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubGameStatusAction, "SubGameStatus");