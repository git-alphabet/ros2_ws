#include "rm_behavior_tree/plugins/rmuc_2026/action/parse_sentry_blackboard.hpp"

namespace rm_behavior_tree
{

ParseSentryBlackboardAction::ParseSentryBlackboardAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus ParseSentryBlackboardAction::tick()
{
  // ── 比赛阶段 ──
  auto game_msg = getInput<rm_decision_interfaces::msg::RMUCGameStatus>("game_status");
  if (game_msg) {
    setOutput("stage_remain_time", static_cast<int>(game_msg->stage_remain_time));
    // 7 分钟赛制: elapsed = 420 - remain
    setOutput("stage_elapsed_time", 420 - static_cast<int>(game_msg->stage_remain_time));
  }

  // ── 机器人状态 ──
  auto robot_ptr = getInput<std::shared_ptr<rm_decision_interfaces::msg::RMUCRobotStatus>>("robot_status");
  if (robot_ptr) {
    const auto & r = **robot_ptr;
    setOutput("hp_cur", static_cast<int>(r.current_hp));
    setOutput("hp_max", static_cast<int>(r.max_hp));
    setOutput("heat_cur", static_cast<int>(r.shooter_heat));
    setOutput("ammo_allow", static_cast<int>(r.ammo_allow));
    setOutput("ammo_left", static_cast<int>(r.ammo_left));
    setOutput("base_hp_cur", static_cast<int>(r.base_hp_cur));
    setOutput("base_hp_max", static_cast<int>(r.base_hp_max));
    setOutput("outpost_alive", r.outpost_alive);
    setOutput("is_dead", r.current_hp <= 0);

    // ── 脱战自计算 (规则: 存活下连续 6s 未发射弹丸且未被扣血) ──
    {
      const int cur_heat = static_cast<int>(r.shooter_heat);
      const int cur_hp   = static_cast<int>(r.current_hp);

      // 获取当前时间
      std::uint64_t now_ms = 0;
      getInput("now_ms", now_ms);

      if (first_tick_) {
        // 首帧: 初始化快照, 比赛开始默认脱战
        // last_combat_ms_ 保持 0 → 距今 > 6s → 脱战
        prev_heat_ = cur_heat;
        prev_hp_   = cur_hp;
        first_tick_ = false;
      } else {
        bool fired   = (cur_heat > prev_heat_);  // 热量上升 → 发射了弹丸
        bool damaged = (cur_hp < prev_hp_ && cur_hp > 0); // 血量下降(且未死) → 被扣血

        if (fired || damaged) {
          last_combat_ms_ = now_ms; // 重置脱战计时
        }
        prev_heat_ = cur_heat;
        prev_hp_   = cur_hp;
      }

      // 死亡时不算脱战
      bool alive = (cur_hp > 0);
      int elapsed_ms = (now_ms > last_combat_ms_)
        ? static_cast<int>(now_ms - last_combat_ms_) : 0;
      bool disengaged = alive && (elapsed_ms >= DISENGAGE_THRESHOLD_MS);

      // 倒计时: 距脱战还差多少秒 (已脱战时为 0)
      int cd_s = disengaged ? 0
        : std::max(0, (DISENGAGE_THRESHOLD_MS - elapsed_ms + 999) / 1000);

      setOutput("is_disengaged", disengaged);
      setOutput("disengage_countdown", cd_s);
    }

    // 经济状态（由上游电控已计算好）
    setOutput("can_remote_heal", r.can_remote_heal);
    setOutput("can_remote_ammo", r.can_remote_ammo);
    setOutput("team_coins", static_cast<int>(r.team_coins));
  }

  // ── 雷达目标 ──
  auto radar_msg = getInput<rm_decision_interfaces::msg::RMUCEnemyTracks>("radar_tracks");
  if (radar_msg && radar_msg->enemy_count > 0) {
    setOutput("has_target", true);
    // 选最近敌方作为 best_target 字符串描述 "id:x:y"
    double px = 0.0, py = 0.0;
    getInput("pose_x", px);
    getInput("pose_y", py);

    double best_dist = 1e9;
    std::string best_target_str = "";
    for (size_t i = 0; i < radar_msg->enemy_count && i < radar_msg->enemy_x.size(); ++i) {
      double dx = radar_msg->enemy_x[i] - px;
      double dy = radar_msg->enemy_y[i] - py;
      double d = std::sqrt(dx * dx + dy * dy);
      if (d < best_dist) {
        best_dist = d;
        best_target_str = std::to_string(radar_msg->enemy_robot_id[i]) + ":" +
                          std::to_string(radar_msg->enemy_x[i]) + ":" +
                          std::to_string(radar_msg->enemy_y[i]);
      }
    }
    setOutput("best_target", best_target_str);

    // 基地威胁判定：任意敌方 y 坐标 ≤ 14m 视为威胁
    bool base_threatened = false;
    for (size_t i = 0; i < radar_msg->enemy_count && i < radar_msg->enemy_y.size(); ++i) {
      if (radar_msg->enemy_y[i] <= 14.0) {
        base_threatened = true;
        break;
      }
    }
    setOutput("base_threat", base_threatened);
    setOutput("fortress_threat", false); // TODO: 结合敌方位置与堡垒距离判定
  } else {
    setOutput("has_target", false);
    setOutput("best_target", std::string(""));
    setOutput("base_threat", false);
    setOutput("fortress_threat", false);
  }

  // ── P0 新增: 0x020D 哨兵决策状态 ──
  auto sds_msg = getInput<rm_decision_interfaces::msg::RMUCSentryDecisionStatus>(
    "sentry_decision_status");
  if (sds_msg) {
    setOutput("can_free_respawn", sds_msg->can_free_respawn);
    setOutput("can_instant_respawn", sds_msg->can_instant_respawn);
    setOutput("instant_respawn_cost", static_cast<int>(sds_msg->instant_respawn_cost));
    setOutput("current_posture", static_cast<int>(sds_msg->current_posture));
    setOutput("remote_ammo_count", static_cast<int>(sds_msg->remote_ammo_count));
    setOutput("remote_heal_count", static_cast<int>(sds_msg->remote_heal_count));
    setOutput("exchanged_ammo_total", static_cast<int>(sds_msg->exchanged_ammo_total));
    setOutput("can_activate_energy", sds_msg->can_activate_energy);
  }

  // ── P0 新增: 0x0204 机器人增益 ──
  auto buff_msg = getInput<rm_decision_interfaces::msg::RMUCRobotBuff>("robot_buff");
  if (buff_msg) {
    setOutput("buff_heal_rate", static_cast<int>(buff_msg->heal_rate));
    setOutput("buff_cool_value", static_cast<int>(buff_msg->cool_value));
    setOutput("buff_defense_pct", static_cast<int>(buff_msg->defense_pct));
    setOutput("buff_vulnerability_pct", static_cast<int>(buff_msg->vulnerability_pct));
    setOutput("buff_attack_pct", static_cast<int>(buff_msg->attack_pct));
  }

  // ── P0 新增: 0x0208 允许发弹量 ──
  auto proj_msg = getInput<rm_decision_interfaces::msg::RMUCProjectileAllowance>(
    "projectile_allowance");
  if (proj_msg) {
    setOutput("fortress_ammo", static_cast<int>(proj_msg->fortress_ammo));
  }

  // ── P0 新增: 0x0101 场地状态 ──
  auto field_msg = getInput<rm_decision_interfaces::msg::RMUCFieldStatus>("field_status");
  if (field_msg) {
    setOutput("field_central_highland", static_cast<int>(field_msg->central_highland));
    setOutput("field_ladder_highland", static_cast<int>(field_msg->ladder_highland));
    setOutput("field_fortress", static_cast<int>(field_msg->fortress));
    setOutput("field_outpost_buff", static_cast<int>(field_msg->outpost_buff));
    setOutput("field_base_buff", field_msg->base_buff);
    setOutput("field_small_energy", static_cast<int>(field_msg->small_energy_status));
    setOutput("field_big_energy", static_cast<int>(field_msg->big_energy_status));
  }

  // ── P0 新增: 0x020C 敌方易伤 ──
  auto mark_msg = getInput<rm_decision_interfaces::msg::RMUCEnemyMark>("enemy_mark");
  if (mark_msg) {
    setOutput("enemy_hero_vuln", mark_msg->enemy_hero_vuln);
    setOutput("enemy_engi_vuln", mark_msg->enemy_engi_vuln);
    setOutput("enemy_infantry3_vuln", mark_msg->enemy_infantry3_vuln);
    setOutput("enemy_infantry4_vuln", mark_msg->enemy_infantry4_vuln);
    setOutput("enemy_sentry_vuln", mark_msg->enemy_sentry_vuln);
  }

  // ── P0 新增: 0x0003 队伍血量 ──
  auto hp_msg = getInput<rm_decision_interfaces::msg::RMUCTeamHP>("team_hp");
  if (hp_msg) {
    setOutput("team_outpost_hp", static_cast<int>(hp_msg->outpost_hp));
    setOutput("team_base_hp", static_cast<int>(hp_msg->base_hp));
  }

  // ── P1: 复活/虚弱状态窗口追踪 ──
  if (robot_ptr) {
    const auto & r = **robot_ptr;
    const bool cur_dead = (r.current_hp <= 0);
    std::uint64_t now_ms = 0;
    getInput("now_ms", now_ms);

    // 1. 复活沿检测: 上帧死亡 → 本帧存活 = 刚复活
    if (prev_dead_ && !cur_dead && now_ms > 0) {
      respawn_ms_ = now_ms;
      // 虚弱解除也会在复活时触发（复活后自带无敌）
      weakness_dispel_ms_ = now_ms;
    }

    // 2. 虚弱解除沿检测: 上帧虚弱 → 本帧不虚弱 → 无敌窗口开始
    //    虚弱 = shooter_power_output == false && alive
    const bool cur_weakness = (!r.shooter_power_output && r.current_hp > 0);
    if (prev_weakness_ && !cur_weakness && !cur_dead && now_ms > 0) {
      weakness_dispel_ms_ = now_ms;
    }
    prev_weakness_ = cur_weakness;

    // 3. 计算复活无敌剩余
    //    无敌来源: 复活 OR 虚弱解除 → 取最近的时刻
    std::uint64_t invincible_start = std::max(respawn_ms_, weakness_dispel_ms_);
    int invincible_elapsed = (now_ms > invincible_start)
      ? static_cast<int>(now_ms - invincible_start) : 0;
    bool is_invincible = !cur_dead && (invincible_elapsed < RESPAWN_INVINCIBLE_MS) &&
                         invincible_start > 0;
    int invincible_remain = is_invincible
      ? (RESPAWN_INVINCIBLE_MS - invincible_elapsed + 999) / 1000 : 0;

    // 4. 功率提升 (仅立即复活后 4s，通过检测 cumulative_instant_count 变化推断)
    //    DecideRespawnCmd 在兑换立即复活时会更新 blackboard 的 cumulative_instant_count
    int bb_cum = 0;
    getInput("cumulative_instant_count", bb_cum);
    if (bb_cum > cumulative_instant_count_) {
      // 检测到新的立即复活兑换
      cumulative_instant_count_ = bb_cum;
      power_boost_ms_ = now_ms;
    }

    int boost_elapsed = (now_ms > power_boost_ms_)
      ? static_cast<int>(now_ms - power_boost_ms_) : 0;
    bool is_boosted = !cur_dead && (boost_elapsed < POWER_BOOST_MS) &&
                      power_boost_ms_ > 0;
    int boost_remain = is_boosted
      ? (POWER_BOOST_MS - boost_elapsed + 999) / 1000 : 0;

    setOutput("is_respawn_invincible", is_invincible);
    setOutput("respawn_invincible_remain_s", invincible_remain);
    setOutput("is_power_boosted", is_boosted);
    setOutput("power_boost_remain_s", boost_remain);
    setOutput("cumulative_instant_count", cumulative_instant_count_);

    prev_dead_ = cur_dead;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::ParseSentryBlackboardAction>("ParseSentryBlackboard");
}
