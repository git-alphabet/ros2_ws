#include "rm_behavior_tree/plugins/rmul_2026/action/init_blackboard_config.hpp"

#include <algorithm>

namespace rm_behavior_tree
{

InitBlackboardConfigAction::InitBlackboardConfigAction(
  const std::string& name,
  const BT::NodeConfig& conf,
  const BT::RosNodeParams& params)
: BT::SyncActionNode(name, conf),
  node_(params.nh)
{
}

BT::NodeStatus InitBlackboardConfigAction::tick()
{
  if (initialized_) {
    return BT::NodeStatus::SUCCESS;
  }

  // 从 ROS 参数读取，YAML 未配置则使用 C++ 默认值
  auto get_param = [this](const std::string& name, double default_val) -> double {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<double>(name, default_val);
    }
    return node_->get_parameter(name).as_double();
  };
  auto get_param_int = [this](const std::string& name, std::int64_t default_val) -> std::uint64_t {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<std::int64_t>(name, default_val);
    }
    return static_cast<std::uint64_t>(node_->get_parameter(name).as_int());
  };

  std::uint64_t heal_wait_ms        = get_param_int("heal_wait_ms", static_cast<std::int64_t>(HEAL_WAIT_MS_DEFAULT));
  double        heal_min_ratio      = get_param("heal_min_ratio", HEAL_MIN_RATIO_DEFAULT);
  std::uint64_t search_timeout_ms   = get_param_int("search_timeout_ms", static_cast<std::int64_t>(SEARCH_TIMEOUT_MS_DEFAULT));
  std::uint64_t recovery_timeout_ms = get_param_int("recovery_timeout_ms", static_cast<std::int64_t>(RECOVERY_TIMEOUT_MS_DEFAULT));
  double        supply_goal_x       = get_param("supply_goal_x", SUPPLY_GOAL_X_DEFAULT);
  double        supply_goal_y       = get_param("supply_goal_y", SUPPLY_GOAL_Y_DEFAULT);
  double        control_goal_x      = get_param("control_goal_x", CONTROL_GOAL_X_DEFAULT);
  double        control_goal_y      = get_param("control_goal_y", CONTROL_GOAL_Y_DEFAULT);
  double        home_goal_x         = get_param("home_goal_x", HOME_GOAL_X_DEFAULT);
  double        home_goal_y         = get_param("home_goal_y", HOME_GOAL_Y_DEFAULT);
  double        arrive_radius       = get_param("arrive_radius", ARRIVE_RADIUS_DEFAULT);

  RCLCPP_INFO(node_->get_logger(),
    "BT Config: supply(%.2f, %.2f) control(%.2f, %.2f) home(%.2f, %.2f) radius=%.2f",
    supply_goal_x, supply_goal_y, control_goal_x, control_goal_y,
    home_goal_x, home_goal_y, arrive_radius);

  // 安全夹紧
  heal_min_ratio = std::clamp(heal_min_ratio, 0.0, 1.0);
  if (arrive_radius < 0.05) {
    arrive_radius = 0.05;
  }

  // 写入黑板 cfg.*
  setOutput("heal_wait_ms", heal_wait_ms);
  setOutput("heal_min_ratio", heal_min_ratio);
  setOutput("search_timeout_ms", search_timeout_ms);
  setOutput("recovery_timeout_ms", recovery_timeout_ms);
  setOutput("supply_goal_x", supply_goal_x);
  setOutput("supply_goal_y", supply_goal_y);
  setOutput("control_goal_x", control_goal_x);
  setOutput("control_goal_y", control_goal_y);
  setOutput("home_goal_x", home_goal_x);
  setOutput("home_goal_y", home_goal_y);
  setOutput("arrive_radius", arrive_radius);

  initialized_ = true;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::InitBlackboardConfigAction, "InitBlackboardConfig");
