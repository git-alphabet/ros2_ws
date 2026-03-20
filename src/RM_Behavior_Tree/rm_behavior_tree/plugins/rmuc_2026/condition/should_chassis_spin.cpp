#include "rm_behavior_tree/plugins/rmuc_2026/condition/should_chassis_spin.hpp"

namespace rm_behavior_tree
{

ShouldChassisSpinCondition::ShouldChassisSpinCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf) {}

BT::NodeStatus ShouldChassisSpinCondition::tick()
{
  double px = 0, py = 0, gx = 0, gy = 0, radius = 0.35;
  int posture = 0;
  bool power_boosted = false, force = false;
  getInput("pose_x", px);
  getInput("pose_y", py);
  getInput("goal_x", gx);
  getInput("goal_y", gy);
  getInput("arrive_radius", radius);
  getInput("current_posture", posture);
  getInput("is_power_boosted", power_boosted);
  getInput("force_spin", force);

  // ★ 最高优先级: 距目标 >= arrive_radius → 正在导航, 绝对禁止旋转 (队伍规定)
  const double dist = std::hypot(gx - px, gy - py);
  if (dist >= radius) return BT::NodeStatus::FAILURE;

  // 以下仅在已到达目标 (站定攻击) 时生效 ──

  // 强制旋转 → 无条件允许
  if (force) return BT::NodeStatus::SUCCESS;

  // 功率翻倍窗口 (立即复活后 4s) → 允许旋转
  if (power_boosted) return BT::NodeStatus::SUCCESS;

  // 移动姿态: 功率 ×1.5 → 小陀螺效果好
  if (posture == 3) return BT::NodeStatus::SUCCESS;

  // 进攻/防御姿态: 功率 ×0.5 → 小陀螺效果差, 不建议旋转
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::ShouldChassisSpinCondition>("ShouldChassisSpin");
}
