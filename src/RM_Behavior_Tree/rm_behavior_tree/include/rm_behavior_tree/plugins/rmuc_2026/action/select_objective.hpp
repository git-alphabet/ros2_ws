#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_OBJECTIVE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_OBJECTIVE_HPP_

#include <string>
#include <cmath>
#include <algorithm>
#include <array>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/**
 * P3 目标控制与区域收益优化 — 多维评分器
 *
 * 候选目标 (8 个):
 *   CENTRAL_HIGHLAND   — 中央高地 (25% 防御, 排他占领)
 *   TRAPEZOIDAL_HIGHLAND — 梯形高地 (50% 防御, 仅己方可占)
 *   BASE_BUFF          — 基地增益点 (50% 防御 + 兑换弹量 + 解除虚弱)
 *   OUTPOST_BUFF       — 前哨站增益点 (25% 防御 + 兑换弹量 + 解除虚弱 + 重建前哨站)
 *   FORTRESS_ALLY      — 己方堡垒 (50% 防御 + 冷却增益 + 储备弹量)
 *   FORTRESS_ENEMY     — 敌方堡垒 (触发对方基地护甲展开, 但 100% 易伤)
 *   SUPPLY_ZONE        — 补给区 (回血 + 免费弹量)
 *   DEFEND_ANCHOR      — 防御锚点 (基地受威胁时回防)
 *
 * 评分维度:
 *   生存收益 + 经济收益 + 防御收益 + 输出收益 - 路径风险(距离)
 *
 * 阶段化策略:
 *   开局 (0~120s):   偏向中央高地 / 梯形高地 / 补给节奏
 *   中盘 (120~300s): 偏向堡垒、前哨站增益点、基地防守
 *   残局 (300~420s): 偏向基地防守、关键补血、保命
 */
class SelectObjectiveAction : public BT::SyncActionNode
{
public:
  SelectObjectiveAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      // ── 位置 ──
      BT::InputPort<double>("pose_x"), BT::InputPort<double>("pose_y"),
      // ── 比赛时间 ──
      BT::InputPort<int>("stage_elapsed_time"),
      BT::InputPort<int>("stage_remain_time"),
      // ── 机器人状态 ──
      BT::InputPort<int>("hp_cur"), BT::InputPort<int>("hp_max"),
      BT::InputPort<int>("ammo_allow", "300", "当前允许发弹量"),
      BT::InputPort<int>("ammo_target", "300", "弹量目标"),
      // ── 基地/前哨站 ──
      BT::InputPort<int>("base_hp_cur"), BT::InputPort<int>("base_hp_max"),
      BT::InputPort<int>("base_deficit_for_fortress"),
      BT::InputPort<bool>("outpost_alive"),
      BT::InputPort<bool>("base_threat"),
      // ── P3 新增: 占领状态 (from 0x0101) ──
      BT::InputPort<int>("field_central_highland", "0", "中央高地占领 0=无 1=己方 2=对方"),
      BT::InputPort<int>("field_ladder_highland", "0", "梯形高地占领"),
      BT::InputPort<int>("field_fortress", "0", "堡垒增益点占领"),
      BT::InputPort<int>("field_outpost_buff", "0", "前哨站增益点占领"),
      BT::InputPort<bool>("field_base_buff", "false", "基地增益点是否被占"),
      // ── P3 新增: 堡垒弹量 ──
      BT::InputPort<int>("fortress_ammo", "0", "堡垒当前储备弹量"),
      // ── 各候选点坐标 ──
      BT::InputPort<double>("central_highland_x", "0.0"),
      BT::InputPort<double>("central_highland_y", "0.0"),
      BT::InputPort<double>("ladder_highland_x", "0.0"),
      BT::InputPort<double>("ladder_highland_y", "0.0"),
      BT::InputPort<double>("base_buff_x", "0.0"),
      BT::InputPort<double>("base_buff_y", "0.0"),
      BT::InputPort<double>("outpost_buff_x", "0.0"),
      BT::InputPort<double>("outpost_buff_y", "0.0"),
      BT::InputPort<double>("fortress_ally_x", "0.0"),
      BT::InputPort<double>("fortress_ally_y", "0.0"),
      BT::InputPort<double>("fortress_enemy_x", "0.0"),
      BT::InputPort<double>("fortress_enemy_y", "0.0"),
      BT::InputPort<double>("buff_zone_x", "0.0"),
      BT::InputPort<double>("buff_zone_y", "0.0"),
      BT::InputPort<double>("defend_anchor_x", "0.0"),
      BT::InputPort<double>("defend_anchor_y", "0.0"),
      // ── outputs ──
      BT::OutputPort<double>("goal_x"), BT::OutputPort<double>("goal_y"),
      BT::OutputPort<std::string>("objective_name")};
  }

  BT::NodeStatus tick() override;

private:
  /// 候选目标枚举
  enum Candidate {
    CENTRAL_HIGHLAND = 0,
    TRAPEZOIDAL_HIGHLAND,
    BASE_BUFF,
    OUTPOST_BUFF,
    FORTRESS_ALLY,
    FORTRESS_ENEMY,
    SUPPLY_ZONE,
    DEFEND_ANCHOR,
    NUM_CANDIDATES
  };

  static constexpr const char * CANDIDATE_NAMES[] = {
    "CENTRAL_HIGHLAND", "TRAPEZOIDAL_HIGHLAND", "BASE_BUFF", "OUTPOST_BUFF",
    "FORTRESS_ALLY", "FORTRESS_ENEMY", "SUPPLY_ZONE", "DEFEND_ANCHOR"
  };
};
}  // namespace rm_behavior_tree
#endif
