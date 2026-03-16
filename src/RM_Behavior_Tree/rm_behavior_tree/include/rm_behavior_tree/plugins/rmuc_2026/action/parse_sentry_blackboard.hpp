#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__PARSE_SENTRY_BLACKBOARD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__PARSE_SENTRY_BLACKBOARD_HPP_

#include <string>
#include <memory>
#include <cmath>
#include "behaviortree_cpp/action_node.h"
#include "rm_decision_interfaces/msg/rmuc_game_status.hpp"
#include "rm_decision_interfaces/msg/rmuc_robot_status.hpp"
#include "rm_decision_interfaces/msg/rmuc_enemy_tracks.hpp"
#include "rm_decision_interfaces/msg/rmuc_sentry_decision_status.hpp"
#include "rm_decision_interfaces/msg/rmuc_robot_buff.hpp"
#include "rm_decision_interfaces/msg/rmuc_projectile_allowance.hpp"
#include "rm_decision_interfaces/msg/rmuc_field_status.hpp"
#include "rm_decision_interfaces/msg/rmuc_enemy_mark.hpp"
#include "rm_decision_interfaces/msg/rmuc_team_positions.hpp"
#include "rm_decision_interfaces/msg/rmuc_team_hp.hpp"

namespace rm_behavior_tree
{
/// 从黑板读取 RMUC 原始消息，解析出派生状态变量写入黑板
class ParseSentryBlackboardAction : public BT::SyncActionNode
{
public:
  ParseSentryBlackboardAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      // inputs (原始消息 — 拆分后各自独立类型)
      BT::InputPort<rm_decision_interfaces::msg::RMUCGameStatus>("game_status"),
      BT::InputPort<std::shared_ptr<rm_decision_interfaces::msg::RMUCRobotStatus>>("robot_status"),
      BT::InputPort<rm_decision_interfaces::msg::RMUCEnemyTracks>("radar_tracks"),
      BT::InputPort<double>("pose_x"),
      BT::InputPort<double>("pose_y"),
      BT::InputPort<std::uint64_t>("now_ms"),
      // P0 新增 inputs: 7 个新话题原始消息
      BT::InputPort<rm_decision_interfaces::msg::RMUCSentryDecisionStatus>("sentry_decision_status"),
      BT::InputPort<rm_decision_interfaces::msg::RMUCRobotBuff>("robot_buff"),
      BT::InputPort<rm_decision_interfaces::msg::RMUCProjectileAllowance>("projectile_allowance"),
      BT::InputPort<rm_decision_interfaces::msg::RMUCFieldStatus>("field_status"),
      BT::InputPort<rm_decision_interfaces::msg::RMUCEnemyMark>("enemy_mark"),
      BT::InputPort<rm_decision_interfaces::msg::RMUCTeamPositions>("team_positions"),
      BT::InputPort<rm_decision_interfaces::msg::RMUCTeamHP>("team_hp"),
      // outputs (派生变量)
      BT::OutputPort<int>("stage_remain_time"),
      BT::OutputPort<int>("stage_elapsed_time"),
      BT::OutputPort<int>("hp_cur"),
      BT::OutputPort<int>("hp_max"),
      BT::OutputPort<int>("heat_cur"),
      BT::OutputPort<int>("ammo_allow"),
      BT::OutputPort<int>("ammo_left"),
      BT::OutputPort<int>("base_hp_cur"),
      BT::OutputPort<int>("base_hp_max"),
      BT::OutputPort<bool>("outpost_alive"),
      BT::OutputPort<bool>("is_dead"),
      BT::OutputPort<bool>("is_disengaged"),
      BT::OutputPort<int>("disengage_countdown"),
      BT::OutputPort<bool>("can_remote_heal"),
      BT::OutputPort<bool>("can_remote_ammo"),
      BT::OutputPort<int>("team_coins"),
      BT::OutputPort<bool>("has_target"),
      BT::OutputPort<std::string>("best_target"),
      BT::OutputPort<bool>("base_threat"),
      BT::OutputPort<bool>("fortress_threat"),
      // P0 新增 outputs: 0x020D 哨兵决策状态
      BT::OutputPort<bool>("can_free_respawn"),
      BT::OutputPort<bool>("can_instant_respawn"),
      BT::OutputPort<int>("instant_respawn_cost"),
      BT::OutputPort<int>("current_posture"),
      BT::OutputPort<int>("remote_ammo_count"),
      BT::OutputPort<int>("remote_heal_count"),
      BT::OutputPort<int>("exchanged_ammo_total"),
      BT::OutputPort<bool>("can_activate_energy"),
      // P0 新增 outputs: 0x0204 增益
      BT::OutputPort<int>("buff_heal_rate"),
      BT::OutputPort<int>("buff_cool_value"),
      BT::OutputPort<int>("buff_defense_pct"),
      BT::OutputPort<int>("buff_vulnerability_pct"),
      BT::OutputPort<int>("buff_attack_pct"),
      // P0 新增 outputs: 0x0208 允许发弹量
      BT::OutputPort<int>("fortress_ammo"),
      // P0 新增 outputs: 0x0101 场地状态
      BT::OutputPort<int>("field_central_highland"),
      BT::OutputPort<int>("field_ladder_highland"),
      BT::OutputPort<int>("field_fortress"),
      BT::OutputPort<int>("field_outpost_buff"),
      BT::OutputPort<bool>("field_base_buff"),
      BT::OutputPort<int>("field_small_energy"),
      BT::OutputPort<int>("field_big_energy"),
      // P0 新增 outputs: 0x020C 敌方易伤
      BT::OutputPort<bool>("enemy_hero_vuln"),
      BT::OutputPort<bool>("enemy_engi_vuln"),
      BT::OutputPort<bool>("enemy_infantry3_vuln"),
      BT::OutputPort<bool>("enemy_infantry4_vuln"),
      BT::OutputPort<bool>("enemy_sentry_vuln"),
      // P0 新增 outputs: 0x0003 队伍血量
      BT::OutputPort<int>("team_outpost_hp"),
      BT::OutputPort<int>("team_base_hp")};
  }

  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree

#endif
