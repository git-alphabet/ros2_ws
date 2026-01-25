#include "rm_behavior_tree/plugins/action/sub_robot_position.hpp"

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

  std::string topic;
  // 使用模板形式以符合 BehaviorTree::TreeNode API
  if (!getInput<std::string>("topic_name", topic)) {
    // 没有输入时使用默认话题（PortsList 中也设置了默认），并记录警告
    RCLCPP_WARN(node_->get_logger(), "SubRobotPositionAction: no topic_name input provided, using default '%s'", topic.c_str());
  }

  // 创建订阅：QoS 缓冲 10，可靠传输（可按需调整）
  rclcpp::QoS qos(10);
  qos.reliable();

  sub_ = node_->create_subscription<rm_interfaces::msg::RobotPosition>(
    topic,
    qos,
    [this](const rm_interfaces::msg::RobotPosition::SharedPtr msg) {
      this->robot_position_callback(msg);
    });
}

void SubRobotPositionAction::robot_position_callback(
  const rm_interfaces::msg::RobotPosition::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 假定 rm_interfaces::msg::RobotPosition 有成员 x, y, yaw
  // 如果你的消息字段不同，请在这里替换为正确字段名
  pose_x_ = msg->x;
  pose_y_ = msg->y;
  pose_yaw_ = msg->yaw;

  // 记录接收时间（如果消息带时间戳并且你希望使用它，可以改为使用 msg->header.stamp）
  last_stamp_ = node_->now();

  has_data_ = true;

  // 尝试将最新位姿直接写入行为树全局黑板，以降低从订阅到黑板更新的延迟。
  // 注意：不同版本的 BehaviorTree.CPP 对黑板并发访问的支持可能不同，请确保在你的环境中这样做是安全的。
  try {
    auto bb = this->config().blackboard;
    if (bb) {
      // 黑板 key 使用 perception 配置中的命名：pose.x, pose.y, pose.yaw
      bb->set("pose.x", pose_x_);
      bb->set("pose.y", pose_y_);
      bb->set("pose.yaw", pose_yaw_);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "SubRobotPositionAction: failed to write to blackboard: %s", e.what());
  }
}

BT::NodeStatus SubRobotPositionAction::tick()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!has_data_) {
    // 尚未收到裁判系统数据，不阻塞行为树，返回 SUCCESS（可根据需要改为 RUNNING/FAILURE）
    return BT::NodeStatus::SUCCESS;
  }

  // 将最新位置写入输出端口
  setOutput("pose_x", pose_x_);
  setOutput("pose_y", pose_y_);
  setOutput("pose_yaw", pose_yaw_);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

// 将节点作为插件导出（放在全局作用域）
CreateRosNodePlugin(
  rm_behavior_tree::SubRobotPositionAction,
  "SubRobotPosition");
