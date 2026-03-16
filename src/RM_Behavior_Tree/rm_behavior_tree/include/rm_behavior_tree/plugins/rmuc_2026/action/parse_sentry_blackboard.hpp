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
      BT::OutputPort<int>("team_base_hp"),
      // P1 新增 outputs: 复活/虚弱状态窗口
      BT::OutputPort<bool>("is_respawn_invincible"),   // 复活后 10s 无敌窗口
      BT::OutputPort<int>("respawn_invincible_remain_s"),  // 无敌剩余秒数
      BT::OutputPort<bool>("is_power_boosted"),        // 立即复活后 4s 功率提升
      BT::OutputPort<int>("power_boost_remain_s"),     // 功率提升剩余秒数
      // P1 双向端口: 读取 DecideRespawnCmd 写入的累计值，自身也输出
      BT::BidirectionalPort<int>("cumulative_instant_count")};
  }

  BT::NodeStatus tick() override;

private:
  // ── 脱战自计算状态 ──
  // 规则: 存活状态下连续 6 秒未发射弹丸且未被扣血 = 脱战
  // 比赛开始时默认脱战，故 last_combat_ms_ 初始化为 0
  int prev_heat_ = 0;          // 上一帧 shooter_heat
  int prev_hp_   = 0;          // 上一帧 current_hp
  std::uint64_t last_combat_ms_ = 0; // 上次交战事件的时间戳(ms)
  bool first_tick_ = true;     // 首次 tick 标记 (跳过首帧差值比较)
  static constexpr int DISENGAGE_THRESHOLD_MS = 6000; // 6 秒

  // ── P1: 复活/虚弱状态窗口追踪 ──
  bool prev_dead_ = true;      // 上一帧死亡状态（初始假设死亡，避免误触发复活沿）
  std::uint64_t respawn_ms_ = 0;       // 复活时刻的时间戳(ms)
  std::uint64_t power_boost_ms_ = 0;   // 功率提升开始时刻(ms)
  int cumulative_instant_count_ = 0;   // 累计立即复活次数
  bool prev_weakness_ = false;         // 上一帧虚弱状态
  std::uint64_t weakness_dispel_ms_ = 0; // 虚弱解除时刻(ms)
  static constexpr int RESPAWN_INVINCIBLE_MS = 10000; // 复活无敌 10s
  static constexpr int POWER_BOOST_MS = 4000;         // 立即复活功率提升 4s
  static constexpr int WEAKNESS_DISPEL_INVINCIBLE_MS = 10000; // 虚弱解除后无敌 10s
};
}  // namespace rm_behavior_tree

#endif
