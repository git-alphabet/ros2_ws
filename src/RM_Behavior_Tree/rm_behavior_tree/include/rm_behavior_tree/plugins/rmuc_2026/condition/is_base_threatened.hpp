#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_BASE_THREATENED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_BASE_THREATENED_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{
/**
 * 基地受威胁判定
 *
 * P3 补充 4: 前哨站存活 → 基地无敌 → 降低威胁敏感度
 *   - 前哨站存活时: 仅当 base_threat==true 且基地血量 < 30% 时才认为受威胁
 *   - 前哨站被击毁后: base_threat==true 或基地血量 < 50% 即认为受威胁
 */
class IsBaseThreatenedCondition : public BT::ConditionNode
{
public:
  IsBaseThreatenedCondition(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("base_threat"),
      BT::InputPort<int>("base_hp_cur"),
      BT::InputPort<int>("base_hp_max"),
      BT::InputPort<double>("enemy_near_base_radius", "2.0", "enemy_near_base_radius"),
      // P3 新增
      BT::InputPort<bool>("outpost_alive", "true", "己方前哨站是否存活")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
