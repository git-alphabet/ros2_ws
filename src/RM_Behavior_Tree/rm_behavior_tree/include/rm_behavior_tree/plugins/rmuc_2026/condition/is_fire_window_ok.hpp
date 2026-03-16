#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_FIRE_WINDOW_OK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_FIRE_WINDOW_OK_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/rmuc_robot_status.hpp"

namespace rm_behavior_tree
{
/// 射击窗口：热量 < 上限 且 弹丸配额 > 0 且 发射机构有供电
/// 注意：不再使用 is_weak bool，改为直接读 robot_status 中的 shooter_power_output
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
        "robot_status", "机器人综合状态消息")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
