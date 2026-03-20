#include "rm_behavior_tree/plugins/rmul_2026/action/get_current_location.hpp"

#include <rclcpp/logging.hpp>

namespace rm_behavior_tree
{

GetCurrentLocationAction::GetCurrentLocationAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf),
  node_(params.nh)
{
  if (!node_) {
    throw std::runtime_error("GetCurrentLocationAction: ROS node is null");
  }

  auto clock = node_->get_clock();
  tf2::Duration buffer_duration(tf2::durationFromSec(10.0));
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock, buffer_duration, node_);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus GetCurrentLocationAction::tick()
{
  std::string map_frame = "map";
  std::string base_frame = "gimbal_yaw";
  getInput("map_frame", map_frame);
  getInput("base_frame", base_frame);

  geometry_msgs::msg::TransformStamped t;

  try {
    t = tf_buffer_->lookupTransform(map_frame, base_frame, tf2::TimePointZero);
    setOutput("current_location", t);

    RCLCPP_DEBUG(
      node_->get_logger(),
      "Current Location: [%.3f, %.3f, %.3f]",
      t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);

    return BT::NodeStatus::SUCCESS;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "GetCurrentLocation TF lookup failed (%s -> %s): %s",
      map_frame.c_str(), base_frame.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::GetCurrentLocationAction, "GetCurrentLocation");
