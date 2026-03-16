#include "rm_behavior_tree/plugins/rmuc_2026/action/decide_respawn_cmd.hpp"
#include <algorithm>

namespace rm_behavior_tree
{

DecideRespawnCmdAction::DecideRespawnCmdAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus DecideRespawnCmdAction::tick()
{
  bool is_dead = false;
  int coins = 0, remain_s = 420, base_hp = 5000, base_max = 5000;
  bool base_threat = false;
  bool can_free = false, can_instant = false;
  int instant_cost = 0;
  int cum_instant = 0;

  getInput("is_dead", is_dead);
  getInput("team_coins", coins);
  getInput("stage_remain_time", remain_s);
  getInput("base_hp_cur", base_hp);
  getInput("base_hp_max", base_max);
  getInput("base_threat", base_threat);
  getInput("can_free_respawn", can_free);
  getInput("can_instant_respawn", can_instant);
  getInput("instant_respawn_cost", instant_cost);
  getInput("cumulative_instant_count", cum_instant);

  int confirm = 0, instant = 0;

  if (is_dead) {
    // ── 1. 始终确认普通复活 ──
    confirm = 1;

    // ── 2. 计算复活倒计时（规则公式） ──
    // countdown = 10 + (420 - remain_s) / 10 + 20 * cumulative_instant_count
    int base_countdown = 10 + (420 - remain_s) / 10 + 20 * cum_instant;
    base_countdown = std::max(base_countdown, 10);  // 最低 10s

    // 当己方基地 HP < 2000 → 倒计时速度 ×4 → 等效等待时间 /4
    int effective_countdown = base_countdown;
    if (base_hp < 2000) {
      effective_countdown = (base_countdown + 3) / 4;  // 向上取整
    }

    // ── 3. 立即复活收益模型 ──
    // 只有当 can_instant == true 且 金币 >= instant_cost 时才考虑
    if (can_instant && coins >= instant_cost && instant_cost > 0) {
      // 基地紧急度得分 (0-100)
      int urgency = 0;
      if (base_threat) urgency += 40;
      if (base_hp < 2000) urgency += 30;
      if (base_hp < 1000) urgency += 20;

      // 时间紧迫度 (比赛末尾)
      if (remain_s < 60) urgency += 30;
      else if (remain_s < 120) urgency += 15;

      // 倒计时等待代价：倒计时越长，立即复活价值越高
      int countdown_penalty = effective_countdown * 3;  // 每秒 3 点

      // 金币成本归一化 (相对于 400 金币的比率)
      int cost_penalty = instant_cost * 100 / 400;

      // 决策: benefit > cost → 兑换
      int benefit = urgency + countdown_penalty;
      if (benefit > cost_penalty) {
        instant = 1;
        // 兑换成功后累加计数 (下一次倒计时会更长)
        setOutput("cumulative_instant_count", cum_instant + 1);
      }
    }

    // ── 4. 兜底: 比赛最后 30 秒 + 有能力 → 强制立即复活 ──
    if (instant == 0 && can_instant && coins >= instant_cost &&
        instant_cost > 0 && remain_s < 30)
    {
      instant = 1;
      setOutput("cumulative_instant_count", cum_instant + 1);
    }
  }

  setOutput("confirm_respawn", confirm);
  setOutput("confirm_instant_respawn", instant);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::DecideRespawnCmdAction>("DecideRespawnCmd");
}
