#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_BEST_TARGET_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_BEST_TARGET_HPP_

#include <string>
#include <cmath>
#include "behaviortree_cpp/action_node.h"
#include "rm_decision_interfaces/msg/rmuc_enemy_tracks.hpp"

namespace rm_behavior_tree
{
/**
 * P3 升级: 综合评分目标选择
 *
 * 选择策略:
 *   score = base_proximity_score + vulnerability_bonus - distance_penalty
 *   - 距离己方基地越近 → base_proximity_score 越高（威胁优先）
 *   - 敌方处于易伤状态 → +30 vulnerability_bonus（补充7: 0x020C 数据）
 *   - 距离自身越远 → distance_penalty 越高（攻击可达性）
 *
 * 如果所有目标评分 ≤ 0 则返回 FAILURE
 */
class SelectBestTargetAction : public BT::SyncActionNode
{
public:
  SelectBestTargetAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rm_decision_interfaces::msg::RMUCEnemyTracks>("radar_tracks"),
      BT::InputPort<double>("pose_x"),
      BT::InputPort<double>("pose_y"),
      BT::InputPort<double>("base_x"),
      BT::InputPort<double>("base_y"),
      // P3 新增: 敌方易伤状态 (补充规则 7, 来自 0x020C)
      BT::InputPort<bool>("enemy_hero_vuln", "false", "英雄易伤"),
      BT::InputPort<bool>("enemy_engi_vuln", "false", "工程易伤"),
      BT::InputPort<bool>("enemy_infantry3_vuln", "false", "3号步兵易伤"),
      BT::InputPort<bool>("enemy_infantry4_vuln", "false", "4号步兵易伤"),
      BT::InputPort<bool>("enemy_sentry_vuln", "false", "哨兵易伤"),
      BT::OutputPort<std::string>("out_target")};
  }
  BT::NodeStatus tick() override;

private:
  /// 根据 robot_id 判断该敌人是否处于易伤状态
  bool isVulnerable(uint8_t robot_id, bool hero, bool engi,
                    bool inf3, bool inf4, bool sentry) const;
};
}  // namespace rm_behavior_tree
#endif
