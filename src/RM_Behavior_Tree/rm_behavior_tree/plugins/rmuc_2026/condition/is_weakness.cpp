#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_weakness.hpp"

#include "rclcpp/logging.hpp"

namespace rm_behavior_tree
{

IsWeaknessCondition::IsWeaknessCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf),
  candidate_frames_(0)
{
}

BT::NodeStatus IsWeaknessCondition::tick()
{
  // ── 1. 读取 robot_status 消息 ──
  auto msg_opt = getInput<std::shared_ptr<rm_decision_interfaces::msg::RMUCRobotStatus>>(
    "robot_status");

  if (!msg_opt || !msg_opt.value()) {
    candidate_frames_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  const auto & r = *msg_opt.value();

  // ── 2. 前置条件：必须存活 ──
  // 死亡状态下 shooter 断电是正常的，不是虚弱
  if (r.current_hp <= 0) {
    candidate_frames_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  // ── 3. shooter 是否有输出 ──
  // 0x0201 bit2: shooter 口 24V 输出，1=有输出(正常)，0=断电(锁定)
  if (r.shooter_power_output) {
    // shooter 正常供电 → 不可能是虚弱
    candidate_frames_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  // ── 到这里：shooter 断电 + 存活 ──
  // ── 4. 排除已知的非虚弱原因 ──

  // 原因 1: 射击热量超限 (shooter_heat >= heat_limit)
  // 裁判系统在热量超限时会锁定发射机构，这不是虚弱
  if (r.shooter_heat >= r.heat_limit && r.heat_limit > 0) {
    candidate_frames_ = 0;
    RCLCPP_DEBUG(rclcpp::get_logger("IsWeakness"),
      "shooter 断电但热量超限 (%u >= %u)，非虚弱",
      r.shooter_heat, r.heat_limit);
    return BT::NodeStatus::FAILURE;
  }

  // 原因 2: 允许发弹量为 0
  // 当允许发弹量耗尽时裁判系统会锁定发射机构
  if (r.ammo_allow == 0) {
    candidate_frames_ = 0;
    RCLCPP_DEBUG(rclcpp::get_logger("IsWeakness"),
      "shooter 断电但允许发弹量为 0，非虚弱");
    return BT::NodeStatus::FAILURE;
  }

  // ── 5. 稳定帧计数：排除射击初速超限 / 测速模块离线等短暂锁定 ──
  // 这些情况在协议中没有直接字段可读，
  // 但属于短暂惩罚（通常 < 5 秒），而虚弱是持续性锁定。
  // 通过连续帧计数来区分：只有稳定超过阈值才判定为虚弱。
  ++candidate_frames_;

  if (candidate_frames_ < STABLE_FRAMES_THRESHOLD) {
    RCLCPP_DEBUG(rclcpp::get_logger("IsWeakness"),
      "候选虚弱帧 %d/%d，等待稳定...",
      candidate_frames_, STABLE_FRAMES_THRESHOLD);
    return BT::NodeStatus::FAILURE;
  }

  // ── 6. 确认虚弱 ──
  RCLCPP_INFO_ONCE(rclcpp::get_logger("IsWeakness"),
    "确认虚弱状态: shooter 断电 + 存活 + 已排除热量超限/弹量为0 + 稳定 %d 帧",
    candidate_frames_);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsWeaknessCondition>("IsWeakness");
}
