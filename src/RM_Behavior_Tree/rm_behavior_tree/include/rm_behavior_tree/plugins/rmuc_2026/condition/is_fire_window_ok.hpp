#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_FIRE_WINDOW_OK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_FIRE_WINDOW_OK_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/rmuc_robot_status.hpp"

namespace rm_behavior_tree
{
/**
 * P4 火力窗口判定 — 从"能打"升级为"值得打"
 *
 * 硬性门槛 (一票否决):
 *   1. 发射机构断电 (虚弱) → FAILURE
 *   2. 弹量 ≤ 0 → FAILURE
 *   3. 热量 ≥ heat_high → FAILURE
 *
 * 软性判断 (值得打):
 *   4. 姿态感知冷却余量: 进攻姿态冷却高 → 热量阈值可放宽
 *   5. 弹量经济性: 弹量 < ammo_conserve 时提高热量门槛 (节约弹药)
 *   6. 易伤状态: 正在易伤时抬高开火条件 (避免浪费弹药在高风险时刻)
 *
 * 补充 2: 进攻姿态 + 堡垒冷却增益 → 冷却 315/s, 可更激进地开火
 * 补充 5: 底盘功率限制 → 与旋转策略关联 (在 CombatLoop 层处理)
 */
class IsFireWindowOkCondition : public BT::ConditionNode
{
public:
  IsFireWindowOkCondition(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("heat_cur"),
      BT::InputPort<int>("heat_high", "210", "heat_high"),
      BT::InputPort<int>("ammo_allow"),
      BT::InputPort<std::shared_ptr<rm_decision_interfaces::msg::RMUCRobotStatus>>(
        "robot_status", "机器人综合状态消息"),
      // P4 新增
      BT::InputPort<int>("current_posture", "0", "当前姿态 1=进攻 2=防御 3=移动"),
      BT::InputPort<int>("buff_cool_value", "0", "0x0204 冷却增益值"),
      BT::InputPort<int>("buff_vulnerability_pct", "0", "0x0204 易伤%"),
      BT::InputPort<int>("ammo_conserve", "30", "弹量节约阈值, 低于此值提高开火条件")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
