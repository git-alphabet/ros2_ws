#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__SHOULD_CHASSIS_SPIN_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__SHOULD_CHASSIS_SPIN_HPP_

#include <string>
#include <cmath>
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{
/**
 * P4 补充 5: 底盘旋转策略
 *
 * ■ 最高优先级规则 (队伍规定):
 *   移动过程中禁止底盘旋转，只有站定不动攻击时才允许开启。
 *   通过 pose 与 goal 的欧氏距离判断: dist >= arrive_radius → 正在导航 → FAILURE
 *
 * ■ 站定时的姿态感知 (次优先级):
 *   进攻/防御姿态: 底盘功率上限 ×0.5 (50W) → 小陀螺效果差
 *   移动姿态:      底盘功率上限 ×1.5 → 小陀螺效果好
 *
 * 决策链:
 *   1. dist(pose, goal) >= arrive_radius → FAILURE (移动中, 绝对禁止旋转)
 *   2. force_spin == true                → SUCCESS (强制旋转)
 *   3. is_power_boosted == true          → SUCCESS (功率翻倍窗口)
 *   4. 移动姿态                          → SUCCESS (功率足够)
 *   5. 进攻/防御姿态                     → FAILURE (功率不足)
 */
class ShouldChassisSpinCondition : public BT::ConditionNode
{
public:
  ShouldChassisSpinCondition(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      // 最高优先级: 通过坐标距离判断是否正在导航 (距目标>=radius → 移动中, 禁止旋转)
      BT::InputPort<double>("pose_x", "0.0", "机器人当前 x"),
      BT::InputPort<double>("pose_y", "0.0", "机器人当前 y"),
      BT::InputPort<double>("goal_x", "0.0", "导航目标 x"),
      BT::InputPort<double>("goal_y", "0.0", "导航目标 y"),
      BT::InputPort<double>("arrive_radius", "0.35", "到达判定半径"),
      // 姿态 & 功率
      BT::InputPort<int>("current_posture", "0", "当前姿态 1=进攻 2=防御 3=移动"),
      BT::InputPort<bool>("is_power_boosted", "false", "立即复活后功率翻倍窗口"),
      BT::InputPort<bool>("force_spin", "false", "强制旋转 (被包围等)")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
