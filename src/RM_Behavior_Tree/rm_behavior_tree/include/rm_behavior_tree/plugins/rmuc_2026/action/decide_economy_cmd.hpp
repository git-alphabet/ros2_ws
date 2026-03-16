#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DECIDE_ECONOMY_CMD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DECIDE_ECONOMY_CMD_HPP_

#include <string>
#include <algorithm>
#include <cmath>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/**
 * P2 经济与补给决策优化节点
 *
 * 功能:
 *   1) 金币预算分层: survival_reserve (为立即复活储备) / sustain / opportunity
 *   2) 远程回血动态成本: cost = 50 + ROUNDUP((420 - remain_s) / 60 * 20)
 *   3) 远程补弹 vs 非远程成本意识: remote 150/100发, non-remote 100/100发
 *   4) 统一收益函数: 在远程回血/远程补弹/allow_ammo/big_energy 之间最优分配
 *   5) 补给周期意识: 每 60s 补给区免费产出 100 发, 接近下一 tick → 倾向物理补给
 *   6) 金币收入节奏: +50/min, 最后一分钟(00:59) +150
 *   7) 堡垒储备弹量公式: N = 100 + 2 * floor(Δ/15), N ≤ 500
 *   8) 0x0120 allow_ammo_target 单调递增约束 (在此处保证)
 */
class DecideEconomyCmdAction : public BT::SyncActionNode
{
public:
  DecideEconomyCmdAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      // ── 机器人状态 ──
      BT::InputPort<int>("hp_cur"),
      BT::InputPort<int>("hp_max"),
      BT::InputPort<int>("ammo_allow"),
      BT::InputPort<int>("ammo_target"),
      BT::InputPort<int>("ammo_low"),
      BT::InputPort<bool>("is_disengaged"),
      BT::InputPort<bool>("can_remote_heal"),
      BT::InputPort<bool>("can_remote_ammo"),
      BT::InputPort<int>("team_coins"),
      BT::InputPort<int>("stage_remain_time"),
      BT::InputPort<bool>("base_threat"),
      // ── P2 新增 inputs ──
      BT::InputPort<int>("instant_respawn_cost", "0", "立即复活金币成本 (from 0x020D)"),
      BT::InputPort<int>("cumulative_instant_count", "0", "累计立即复活次数"),
      BT::InputPort<int>("base_hp_cur", "5000", "己方基地当前 HP"),
      BT::InputPort<int>("base_hp_max", "5000", "己方基地最大 HP"),
      BT::InputPort<int>("fortress_ammo", "0", "堡垒当前弹丸储量 (from 0x0208)"),
      BT::InputPort<int>("remote_heal_count", "0", "累计远程回血次数 (from 0x020D)"),
      BT::InputPort<int>("remote_ammo_count", "0", "累计远程补弹次数 (from 0x020D)"),
      // ── 双向 (allow_ammo 单调递增) ──
      BT::InputPort<int>("allow_ammo_target_in"),
      BT::OutputPort<int>("allow_ammo_target_out"),
      // ── outputs ──
      BT::OutputPort<int>("trigger_remote_ammo"),
      BT::OutputPort<int>("trigger_remote_hp"),
      BT::OutputPort<int>("enable_big_energy")};
  }

  BT::NodeStatus tick() override;

private:
  // ── 远程操作帧内去重 (上升沿控制) ──
  int prev_remote_heal_count_ = 0;
  int prev_remote_ammo_count_ = 0;

  // ── 补充规则常量 ──
  static constexpr int REMOTE_HEAL_BASE_COST = 50;       // 远程回血基础金币
  static constexpr int REMOTE_AMMO_COST = 150;           // 远程补弹每次 150 金币 / 100 发
  static constexpr int NONREMOTE_AMMO_COST_PER_100 = 100;// 非远程 100 金币 / 100 发
  static constexpr int SUPPLY_TICK_FREE_AMMO = 100;      // 补给区每 60s 免费产出 100 发
  static constexpr int FORTRESS_AMMO_MAX = 500;           // 堡垒最大弹量
  static constexpr int COIN_INCOME_PER_MIN = 50;          // 每分钟正常金币收入
  static constexpr int COIN_LAST_MINUTE_BONUS = 150;      // 最后一分钟(00:59) 额外收入
};
}  // namespace rm_behavior_tree
#endif
