#include "rm_behavior_tree/plugins/rmul_2026/condition/is_nav_timeout.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rm_behavior_tree
{

IsNavTimeout::IsNavTimeout(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config),
  timer_started_(false)
{
}

BT::NodeStatus IsNavTimeout::tick()
{
  // 如果 nav_status 指示已到达目标，重置计时器并返回 FAILURE（不需要 recovery）
  auto nav_res = getInput<bool>("nav_status");
  if (nav_res.has_value() && nav_res.value()) {
    timer_started_ = false;
    return BT::NodeStatus::FAILURE;
  }

  // 从黑板读取 ROS 时间（支持 use_sim_time）
  std::uint64_t now_ms = 0;
  if (auto res = getInput<std::uint64_t>("now_ms"); res.has_value()) {
    now_ms = res.value();
  } else {
    // 兆底：使用系统墙钟（实车场景 now_ms 可能未设置）
    now_ms = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  // 首次未到达目标时启动计时器
  if (!timer_started_) {
    start_time_ms_ = now_ms;
    timer_started_ = true;
    RCLCPP_DEBUG(
      rclcpp::get_logger("rm_behavior_tree"),
      "[IsNavTimeout] Timer started (ROS time)");
    return BT::NodeStatus::FAILURE;
  }

  double timeout_sec = 30.0;
  auto timeout_res = getInput<double>("timeout_sec");
  if (timeout_res.has_value()) {
    timeout_sec = timeout_res.value();
  }

  // 溢出保护：时间回跳时重置计时器
  const double elapsed_sec = (now_ms >= start_time_ms_)
    ? static_cast<double>(now_ms - start_time_ms_) / 1000.0
    : 0.0;

  if (elapsed_sec >= timeout_sec) {
    RCLCPP_WARN(
      rclcpp::get_logger("rm_behavior_tree"),
      "[IsNavTimeout] Navigation timeout! elapsed=%.1fs >= threshold=%.1fs, triggering recovery",
      elapsed_sec, timeout_sec);
    // 重置计时器，为下一轮 recovery 周期做准备
    timer_started_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsNavTimeout>("IsNavTimeout");
}
