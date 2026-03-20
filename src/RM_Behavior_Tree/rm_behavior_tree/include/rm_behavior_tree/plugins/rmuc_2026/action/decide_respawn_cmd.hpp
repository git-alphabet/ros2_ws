#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DECIDE_RESPAWN_CMD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DECIDE_RESPAWN_CMD_HPP_

#include <string>
#include <memory>
#include <algorithm>
#include "behaviortree_cpp/action_node.h"
#include "rm_decision_interfaces/msg/rmuc_robot_status.hpp"

namespace rm_behavior_tree
{
/**
 * 根据死亡状态、经济、比赛时间、复活公式决定是否确认复活 / 兑换立即复活。
 *
 * 复活倒计时公式（RMUC 2026 规则）:
 *   countdown = 10 + (420 - remain_s) / 10 + 20 * cumulative_instant_count
 *   当己方基地 HP < 2000 → 倒计时速度 ×4 → 等效 countdown /= 4
 *
 * 立即复活收益模型:
 *   benefit = 倒计时等待时间的期望伤害减少 + 基地威胁紧迫度
 *   cost   = instant_respawn_cost (金币)
 *   → 当 benefit > cost 或紧急条件满足时兑换
 */
class DecideRespawnCmdAction : public BT::SyncActionNode
{
public:
  DecideRespawnCmdAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("is_dead"),
      BT::InputPort<std::shared_ptr<rm_decision_interfaces::msg::RMUCRobotStatus>>("robot_status"),
      BT::InputPort<int>("team_coins"),
      BT::InputPort<int>("stage_remain_time"),
      BT::InputPort<int>("base_hp_cur"),
      BT::InputPort<int>("base_hp_max"),
      BT::InputPort<bool>("base_threat"),
      // P1 新增: 从 0x020D 获取的复活可用性
      BT::InputPort<bool>("can_free_respawn"),
      BT::InputPort<bool>("can_instant_respawn"),
      BT::InputPort<int>("instant_respawn_cost"),
      // P1 新增: 累计立即复活次数 (黑板跟踪，影响倒计时公式)
      BT::BidirectionalPort<int>("cumulative_instant_count"),
      // outputs
      BT::OutputPort<int>("confirm_respawn"),
      BT::OutputPort<int>("confirm_instant_respawn")};
  }

  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree

#endif
