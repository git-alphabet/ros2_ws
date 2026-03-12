#include "rm_behavior_tree/plugins/rmul_2026/action/sub_robot_position.hpp"

#include "behaviortree_ros2/plugins.hpp"  // CreateRosNodePlugin 宏

namespace rm_behavior_tree
{

SubRobotPositionAction::SubRobotPositionAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf),
  node_(params.nh)
{
  if (!node_) {
    throw std::runtime_error("SubRobotPositionAction: ROS node is null");
  }

  rclcpp::QoS qos(10);
  qos.reliable();

  // 订阅 RMULNav：读 is_at_nav_goal, is_detect_enemy
  std::string nav_topic;
  if (!getInput<std::string>("topic_name", nav_topic)) {
    RCLCPP_WARN(node_->get_logger(), "SubRobotPositionAction: no topic_name provided, using default '%s'", nav_topic.c_str());
  }
  nav_sub_ = node_->create_subscription<rm_decision_interfaces::msg::RMULNav>(
    nav_topic, qos,
    [this](const rm_decision_interfaces::msg::RMULNav::SharedPtr msg) {
      this->nav_callback(msg);
    });

  // 订阅 RMULRob：读位姿 x, y
  std::string rob_topic;
  if (!getInput<std::string>("rob_topic_name", rob_topic)) {
    RCLCPP_WARN(node_->get_logger(), "SubRobotPositionAction: no rob_topic_name provided, using default '%s'", rob_topic.c_str());
  }
  rob_sub_ = node_->create_subscription<rm_decision_interfaces::msg::RMULRob>(
    rob_topic, qos,
    [this](const rm_decision_interfaces::msg::RMULRob::SharedPtr msg) {
      this->rob_callback(msg);
    });
}

void SubRobotPositionAction::nav_callback(
  const rm_decision_interfaces::msg::RMULNav::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  is_at_nav_goal_ = msg->is_at_nav_goal;
  is_detect_enemy_ = msg->is_detect_enemy;
  last_stamp_ = node_->now();
  has_nav_data_ = true;
}

void SubRobotPositionAction::rob_callback(
  const rm_decision_interfaces::msg::RMULRob::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pose_x_ = static_cast<double>(msg->x);
  pose_y_ = static_cast<double>(msg->y);
  has_rob_data_ = true;
}

BT::NodeStatus SubRobotPositionAction::tick()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!has_nav_data_ || !has_rob_data_) {
    // 尚未收到全部数据，不阻塞行为树
    return BT::NodeStatus::SUCCESS;
  }

  setOutput("pose_x", pose_x_);
  setOutput("pose_y", pose_y_);
  setOutput("is_at_nav_goal", is_at_nav_goal_);
  setOutput("is_detect_enemy", is_detect_enemy_);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

// 将节点作为插件导出（放在全局作用域）
CreateRosNodePlugin(
  rm_behavior_tree::SubRobotPositionAction,
  "SubRobotPosition");
