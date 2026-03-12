#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_AT_NAV_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_AT_NAV_GOAL_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
// is_at_nav_goal 直接从黑板读 bool，无需引入 msg 头文件

namespace rm_behavior_tree
{

/**
 * @brief 判断机器人是否到达导航目标点
 *
 * 黑板输入：
 *   - nav_status: rm_decision_interfaces::msg::RMULNav
 *
 * 语义：
 *   - is_at_nav_goal == true  -> SUCCESS（已到达目标点）
 *   - is_at_nav_goal == false -> FAILURE（未到达目标点）
 *   - 若读不到 nav_status -> FAILURE（保守失败）
 *
 * 设计说明：
 *  - 基于 RMUL_nav.msg 的 is_at_nav_goal 字段进行判断
 *  - 读不到输入时选择 FAILURE，避免订阅链路断线导致误判
 */
class IsAtNavGoal : public BT::ConditionNode
{
public:
  IsAtNavGoal(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("nav_status")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_AT_NAV_GOAL_HPP_
