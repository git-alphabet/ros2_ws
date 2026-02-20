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
  auto game_msg = getInput<rm_decision_interfaces::msg::RMUC>("game_status");
  if (game_msg) {
    setOutput("stage_remain_time", static_cast<int>(game_msg->stage_remain_time));
    // 7 分钟赛制: elapsed = 420 - remain
    setOutput("stage_elapsed_time", 420 - static_cast<int>(game_msg->stage_remain_time));
  }

  // ── 机器人状态 ──
  auto robot_ptr = getInput<std::shared_ptr<rm_decision_interfaces::msg::RMUC>>("robot_status");
  if (robot_ptr) {
    const auto & r = **robot_ptr;
    setOutput("hp_cur", static_cast<int>(r.current_hp));
    setOutput("hp_max", static_cast<int>(r.hp_max));
    setOutput("heat_cur", static_cast<int>(r.shooter_heat));
    setOutput("ammo_allow", static_cast<int>(r.projectile_allowance));
    setOutput("ammo_left", static_cast<int>(r.remaining_ammo));
    setOutput("base_hp_cur", static_cast<int>(r.base_hp));
    setOutput("base_hp_max", 5000);  // RMUC 基地满血 5000
    setOutput("outpost_alive", r.outpost_hp > 0);
    setOutput("is_dead", r.current_hp <= 0);
    setOutput("is_weak", r.is_weak);

    // 脱战判定（简化：血量一定时间未减少则脱战，此处由上游直接提供或默认 false）
    setOutput("is_disengaged", false);
    setOutput("disengage_countdown", 0);

    // 经济状态
    setOutput("can_remote_heal", r.gold_coins >= 200);  // 200 金币可远程回血
    setOutput("can_remote_ammo", r.gold_coins >= 100);  // 100 金币可远程补弹
    setOutput("team_coins", static_cast<int>(r.gold_coins));
  }

  // ── 雷达目标 ──
  auto radar_msg = getInput<rm_decision_interfaces::msg::RMUC>("radar_tracks");
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
    setOutput("base_threat", false);     // TODO: 结合敌方位置与基地距离判定
    setOutput("fortress_threat", false); // TODO: 结合敌方位置与堡垒距离判定
  } else {
    setOutput("has_target", false);
    setOutput("best_target", std::string(""));
    setOutput("base_threat", false);
    setOutput("fortress_threat", false);
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::ParseSentryBlackboardAction>("ParseSentryBlackboard");
}
