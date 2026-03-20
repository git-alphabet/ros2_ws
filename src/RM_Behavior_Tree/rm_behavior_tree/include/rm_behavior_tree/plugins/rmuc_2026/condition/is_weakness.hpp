#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_WEAKNESS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_WEAKNESS_HPP_

// =============================================================================
// IsWeakness 条件节点 — 基于协议字段组合推断机器人是否处于虚弱状态
// =============================================================================
//
// 规则依据 (rmuc_rule.md 第 4 页):
//   读条复活后进入"虚弱"状态: 发射机构锁定 + 无法占领增益点 + 无法重建前哨站。
//   虚弱解除: 检测到可占领的前哨站/基地增益点/补给区增益点 RFID 卡。
//
// 判定逻辑:
//   虚弱 = shooter 断电 (0x0201 bit2 == 0)
//        + 当前存活 (hp > 0)
//        + 排除以下非虚弱的 shooter 断电原因:
//          1. 射击热量超限: shooter_heat >= heat_limit
//          2. 允许发弹量为 0: ammo_allow == 0
//
//   关于射击初速度超限 / 测速模块离线:
//     这两种情况在协议中没有直接状态字段可读,
//     但它们属于**短暂惩罚性锁定**(通常 < 5 秒),
//     而虚弱是**持续性锁定**(直到刷卡解除)。
//     通过稳定帧计数(STABLE_FRAMES_THRESHOLD)排除短暂锁定:
//     只有连续多帧都满足"shooter断电+存活+非热量超限+非弹量为0"
//     才判定为虚弱,避免初速超限/测速离线造成的短暂误判。
//
// 数据来源:
//   robot_status (RMUCRobotStatus):
//     - shooter_power_output: 0x0201 bit2  (shooter 24V 输出)
//     - current_hp:           0x0201 offset 2
//     - shooter_heat:         0x0202 offset 10
//     - heat_limit:           0x0201 offset 8
//     - ammo_allow:           0x0201 (或 0x0208 offset 0)
// =============================================================================

#include <string>
#include <memory>
#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/rmuc_robot_status.hpp"

namespace rm_behavior_tree
{

class IsWeaknessCondition : public BT::ConditionNode
{
public:
  IsWeaknessCondition(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<rm_decision_interfaces::msg::RMUCRobotStatus>>(
        "robot_status", "机器人综合状态消息")
    };
  }

  BT::NodeStatus tick() override;

private:
  // 连续满足"候选虚弱"条件的帧数
  int candidate_frames_ = 0;

  // 当连续 N 帧都满足候选条件时才判定为虚弱，
  // 用于排除射击初速度超限 / 测速模块离线等短暂锁定（通常 < 5s，即 < 50 帧 @10Hz）
  // 设置为 20 帧（@10Hz ≈ 2s），在绝大多数短暂惩罚结束后不会误判，
  // 同时不会让真正的虚弱检测延迟过长。
  static constexpr int STABLE_FRAMES_THRESHOLD = 20;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_WEAKNESS_HPP_
