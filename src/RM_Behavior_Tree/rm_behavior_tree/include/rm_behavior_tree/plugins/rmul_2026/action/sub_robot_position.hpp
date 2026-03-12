#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ROBOT_POSITION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ROBOT_POSITION_HPP_

#include <mutex>
#include <string>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"

#include "rm_decision_interfaces/msg/rmul_nav.hpp"
#include "rm_decision_interfaces/msg/rmul_rob.hpp"

namespace rm_behavior_tree
{

class SubRobotPositionAction : public BT::SyncActionNode
{
public:
  SubRobotPositionAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  /// BehaviorTree 要求的静态端口声明
  static BT::PortsList providedPorts()
  {
    return {
      // nav_topic_name: 订阅 RMULNav（导航状态：is_at_nav_goal, is_detect_enemy）
      BT::InputPort<std::string>("topic_name", std::string("/robot_position"), "RMULNav 话题名"),
      // rob_topic_name: 订阅 RMULRob（位姿：x, y）
      BT::InputPort<std::string>("rob_topic_name", std::string("/robot_status"), "RMULRob 话题名（读取位姿 x/y）"),
      BT::OutputPort<double>("pose_x"),
      BT::OutputPort<double>("pose_y"),
      BT::OutputPort<bool>("is_at_nav_goal"),
      BT::OutputPort<bool>("is_detect_enemy")
    };
  }

  BT::NodeStatus tick() override;

private:
  void nav_callback(
    const rm_decision_interfaces::msg::RMULNav::SharedPtr msg);
  void rob_callback(
    const rm_decision_interfaces::msg::RMULRob::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<rm_decision_interfaces::msg::RMULNav>::SharedPtr nav_sub_;
  rclcpp::Subscription<rm_decision_interfaces::msg::RMULRob>::SharedPtr rob_sub_;

  mutable std::mutex mutex_;

  // 最新一次接收到的数据
  double pose_x_{0.0};
  double pose_y_{0.0};
  bool is_at_nav_goal_{false};
  bool is_detect_enemy_{false};
  rclcpp::Time last_stamp_;

  bool has_nav_data_{false};
  bool has_rob_data_{false};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ROBOT_POSITION_HPP_
