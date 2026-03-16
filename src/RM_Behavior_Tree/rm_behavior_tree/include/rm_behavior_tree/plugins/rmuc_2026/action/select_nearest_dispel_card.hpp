#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_NEAREST_DISPEL_CARD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_NEAREST_DISPEL_CARD_HPP_

#include <string>
#include <cmath>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{

/**
 * SelectNearestBuffZone
 * 共用逻辑：从 buff_zone（补给区）/ base_buff（基地增益区）/ outpost_buff（前哨站增益区）
 * 三个候选点中选出距自身最近的一个，输出到 goal_x/y。
 *
 * 场景 A —— 申弱解除（WeaknessRecovery）：
 *   名称: SelectNearestDispelCard
 *   目的: 虹弱时寻找最近 RFID 刷卡点以解除 shooter 断电状态
 *
 * 场景 B —— 补弹（AmmoPlan）：
 *   名称: SelectNearestResupplyStation
 *   目的: 缺弹时寻找最近补弹点（同一组候选址，因为三个區均支持补弹）
 */
class SelectNearestBuffZoneAction : public BT::SyncActionNode
{
public:
  SelectNearestBuffZoneAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("pose_x"), BT::InputPort<double>("pose_y"),
      BT::InputPort<double>("buff_zone_x"), BT::InputPort<double>("buff_zone_y"),
      BT::InputPort<double>("base_buff_x"), BT::InputPort<double>("base_buff_y"),
      BT::InputPort<double>("outpost_buff_x"), BT::InputPort<double>("outpost_buff_y"),
      BT::OutputPort<double>("goal_x"), BT::OutputPort<double>("goal_y")};
  }
  BT::NodeStatus tick() override;
};

/// 场景 A 别名：用于 WeaknessRecovery 子树——申弱时寻找最近 RFID 刷卡点
using SelectNearestDispelCardAction    = SelectNearestBuffZoneAction;
/// 场景 B 别名：用于 AmmoPlan 子树——缺弹时寻找最近补弹点
using SelectNearestResupplyStationAction = SelectNearestBuffZoneAction;

}  // namespace rm_behavior_tree
#endif
