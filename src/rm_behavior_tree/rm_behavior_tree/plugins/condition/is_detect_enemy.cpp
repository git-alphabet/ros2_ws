#include "rm_behavior_tree/plugins/condition/is_detect_enemy.hpp"

namespace rm_behavior_tree
{

IsDetectEnemyAction::IsDetectEnemyAction(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsDetectEnemyAction::detectEnemyStatus, this), config)
{
}

BT::NodeStatus IsDetectEnemyAction::detectEnemyStatus()
{
  // 1. 获取输入的RMUL.msg数据
  auto msg = getInput<auto_aim_interfaces::msg::RMUL>("message");

  if (!msg) {
    std::cerr << "Missing required input [message]" << '\n';
    return BT::NodeStatus::FAILURE;
  }

  // 2. 数据时间戳对齐：判断数据是否在有效时间范围内（核心新增逻辑）
  // 2.1 获取当前时刻（ROS2当前时间）
  rclcpp::Time current_time = rclcpp::Clock().now();
  // 2.2 获取msg的生成时间戳（从header.stamp中提取）
  rclcpp::Time msg_time(msg->header.stamp.sec, msg->header.stamp.nanosec);
  // 2.3 计算时间差（单位：毫秒ms），此处设置有效阈值为100ms（可根据你的系统调整）
  const int64_t valid_time_threshold = 100; // 有效数据超时时间：100毫秒
  int64_t time_diff_ms = (current_time - msg_time).nanoseconds() / 1000000;

  // 2.4 过滤过期数据：如果时间差超过阈值，直接返回FAILURE，不使用该数据
  if (time_diff_ms > valid_time_threshold || time_diff_ms < 0) {
    std::cerr << "RMUL data is expired! Time diff: " << time_diff_ms << "ms (threshold: " << valid_time_threshold << "ms)" << '\n';
    return BT::NodeStatus::FAILURE;
  }

  // 3. 原有逻辑：判断是否检测到敌人（仅使用有效时间范围内的数据）
  if (msg->is_detect_enemy) {
    // 检测到敌人（且数据未过期）
    return BT::NodeStatus::SUCCESS;
  } else {
    // 未检测到敌人
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsDetectEnemyAction>("IsDetectEnemy");
}
