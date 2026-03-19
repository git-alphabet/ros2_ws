#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__GET_CURRENT_LOCATION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__GET_CURRENT_LOCATION_HPP_

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <rclcpp/rclcpp.hpp>

namespace rm_behavior_tree
{

class GetCurrentLocationAction : public BT::SyncActionNode
{
public:
  GetCurrentLocationAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::TransformStamped>("current_location"),
      BT::InputPort<std::string>("map_frame", "map", "TF map frame name"),
      BT::InputPort<std::string>("base_frame", "gimbal_yaw", "TF base frame name")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__GET_CURRENT_LOCATION_HPP_
