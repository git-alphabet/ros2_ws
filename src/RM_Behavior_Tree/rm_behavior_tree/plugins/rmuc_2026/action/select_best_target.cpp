#include "rm_behavior_tree/plugins/rmuc_2026/action/select_best_target.hpp"

namespace rm_behavior_tree
{

SelectBestTargetAction::SelectBestTargetAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

bool SelectBestTargetAction::isVulnerable(
  uint8_t robot_id, bool hero, bool engi,
  bool inf3, bool inf4, bool sentry) const
{
  // RMUC 红方 ID: 1=英雄, 2=工程, 3/4=步兵, 7=哨兵
  // RMUC 蓝方 ID: 101=英雄, 102=工程, 103/104=步兵, 107=哨兵
  uint8_t role = (robot_id > 100) ? (robot_id - 100) : robot_id;
  switch (role) {
    case 1: return hero;
    case 2: return engi;
    case 3: return inf3;
    case 4: return inf4;
    case 7: return sentry;
    default: return false;
  }
}

BT::NodeStatus SelectBestTargetAction::tick()
{
  auto radar = getInput<rm_decision_interfaces::msg::RMUCEnemyTracks>("radar_tracks");
  if (!radar || radar->enemy_count == 0) {
    setOutput("out_target", std::string(""));
    return BT::NodeStatus::FAILURE;
  }

  double px = 0, py = 0, bx = 0, by = 0;
  getInput("pose_x", px);
  getInput("pose_y", py);
  getInput("base_x", bx);
  getInput("base_y", by);

  // P3 补充7: 读取敌方易伤状态
  bool hero_v = false, engi_v = false, inf3_v = false, inf4_v = false, sentry_v = false;
  getInput("enemy_hero_vuln", hero_v);
  getInput("enemy_engi_vuln", engi_v);
  getInput("enemy_infantry3_vuln", inf3_v);
  getInput("enemy_infantry4_vuln", inf4_v);
  getInput("enemy_sentry_vuln", sentry_v);

  // 综合评分: 选择得分最高的目标
  int best_idx = -1;
  double best_score = -1e9;

  for (size_t i = 0; i < radar->enemy_count && i < radar->enemy_x.size(); ++i) {
    double ex = radar->enemy_x[i];
    double ey = radar->enemy_y[i];
    double d_base = std::hypot(ex - bx, ey - by);
    double d_self = std::hypot(ex - px, ey - py);

    // --- 基地威胁分 ---
    // 距基地越近，基础分越高 (满分 50 @ 0m, 0 @ 10m+)
    double base_score = std::max(0.0, 50.0 - d_base * 5.0);

    // --- 易伤加成 (补充规则 7) ---
    // 易伤目标受到的伤害增加100%，打它收益翻倍
    double vuln_bonus = 0.0;
    if (i < radar->enemy_robot_id.size()) {
      if (isVulnerable(radar->enemy_robot_id[i], hero_v, engi_v, inf3_v, inf4_v, sentry_v)) {
        vuln_bonus = 30.0;
      }
    }

    // --- 距离惩罚 ---
    // 离自身太远难以有效攻击 (线性 -2/m)
    double dist_penalty = d_self * 2.0;

    // --- 置信度加成 ---
    double conf_bonus = 0.0;
    if (i < radar->enemy_confidence.size()) {
      conf_bonus = radar->enemy_confidence[i] * 10.0;  // [0, 10]
    }

    double score = base_score + vuln_bonus - dist_penalty + conf_bonus;

    if (score > best_score) {
      best_score = score;
      best_idx = static_cast<int>(i);
    }
  }

  if (best_idx < 0) {
    setOutput("out_target", std::string(""));
    return BT::NodeStatus::FAILURE;
  }

  std::string target_str =
    std::to_string(radar->enemy_robot_id[best_idx]) + ":" +
    std::to_string(radar->enemy_x[best_idx]) + ":" +
    std::to_string(radar->enemy_y[best_idx]);
  setOutput("out_target", target_str);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::SelectBestTargetAction>("SelectBestTarget");
}
