#include "rm_behavior_tree/plugins/rmuc_2026/action/decide_economy_cmd.hpp"

namespace rm_behavior_tree
{

DecideEconomyCmdAction::DecideEconomyCmdAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus DecideEconomyCmdAction::tick()
{
  // ══════════════════════════════════════════════════════
  // 0. 读取所有输入
  // ══════════════════════════════════════════════════════
  int hp = 400, hp_max = 400, ammo = 300, ammo_target = 300, ammo_low = 80;
  bool disengaged = false, can_heal = false, can_ammo = false, base_threat = false;
  int coins = 0, remain_s = 420;
  int instant_cost = 0, cum_instant = 0;
  int base_hp = 5000, base_max = 5000;
  int fortress_ammo = 0;
  int remote_heal_cnt = 0, remote_ammo_cnt = 0;

  getInput("hp_cur", hp);
  getInput("hp_max", hp_max);
  getInput("ammo_allow", ammo);
  getInput("ammo_target", ammo_target);
  getInput("ammo_low", ammo_low);
  getInput("is_disengaged", disengaged);
  getInput("can_remote_heal", can_heal);
  getInput("can_remote_ammo", can_ammo);
  getInput("team_coins", coins);
  getInput("stage_remain_time", remain_s);
  getInput("base_threat", base_threat);
  getInput("instant_respawn_cost", instant_cost);
  getInput("cumulative_instant_count", cum_instant);
  getInput("base_hp_cur", base_hp);
  getInput("base_hp_max", base_max);
  getInput("fortress_ammo", fortress_ammo);
  getInput("remote_heal_count", remote_heal_cnt);
  getInput("remote_ammo_count", remote_ammo_cnt);

  int trig_ammo = 0, trig_hp = 0, big_energy = 0;
  int allow_ammo = 0;
  getInput("allow_ammo_target_in", allow_ammo);

  // ══════════════════════════════════════════════════════
  // 1. 金币预算分层 (P2 优化 1)
  // ══════════════════════════════════════════════════════
  //   survival_reserve: 为下一次立即复活保留的金币
  //   available_coins:  扣除 reserve 后可用于日常经济决策的金币
  //   expected_income:  预计未来还能收入的金币 (P2 补充 4)
  //
  //   金币收入节奏 (补充规则 4):
  //     - 每分钟 +50 (正常)
  //     - 最后一分钟 (remain_s ≤ 60) 额外 +150
  int survival_reserve = 0;
  if (instant_cost > 0) {
    // 保留足够下一次立即复活的金币
    survival_reserve = instant_cost;
  } else {
    // 未知 cost → 保守估算: 基础公式 = 50 + 累计次数影响
    // 但无法精确知道，取保守值 200
    survival_reserve = 200;
  }

  // 预计未来金币收入 (补充 4)
  int remaining_full_minutes = remain_s / 60;  // 还有几个完整分钟
  int expected_income = remaining_full_minutes * COIN_INCOME_PER_MIN;
  if (remain_s > 0 && remain_s <= 60) {
    expected_income += COIN_LAST_MINUTE_BONUS;  // 最后一分钟的 +150
  } else if (remain_s > 60) {
    expected_income += COIN_LAST_MINUTE_BONUS;  // 未来最后一分钟还会有 +150
  }

  // 可用于经济决策的预算 = 当前金币 - survival_reserve
  int available = std::max(0, coins - survival_reserve);

  // 如果比赛剩余 < 30s，不再保留 reserve（不可能再死一次有意义的复活了）
  if (remain_s < 30) {
    available = coins;
  }

  // ══════════════════════════════════════════════════════
  // 2. 远程回血决策 (P2 优化 3 + 补充 1)
  // ══════════════════════════════════════════════════════
  //   动态成本公式 (补充规则 1):
  //     cost = 50 + ROUNDUP((420 - remain_s) / 60 * 20)
  //   更保守触发条件:
  //     - 必须脱战 (规则硬性要求)
  //     - HP < 40% (从 50% 收紧到 40%)
  //     - 金币预算充足 (扣除 reserve 后仍够)
  //     - 成本效益: 恢复的 HP 价值 > 金币成本
  int remote_heal_cost = 0;
  int heal_score = 0;  // 用于统一收益函数
  if (disengaged && can_heal && hp_max > 0) {
    // 计算动态成本 (补充 1)
    int elapsed_s = 420 - remain_s;
    remote_heal_cost = REMOTE_HEAL_BASE_COST +
      static_cast<int>(std::ceil(static_cast<double>(elapsed_s) / 60.0 * 20.0));

    // HP 缺失比例
    double hp_ratio = static_cast<double>(hp) / hp_max;

    // 收益: HP 恢复带来的生存价值
    // 远程回血恢复量 = 10% max_hp (RMUC 规则), 归一化到 0-100 分
    int heal_amount = hp_max / 10;
    int hp_missing = hp_max - hp;
    int effective_heal = std::min(heal_amount, hp_missing);

    // 生存收益 = (恢复量 / max_hp) * 200 + 血量越低加分越多
    heal_score = effective_heal * 200 / std::max(1, hp_max);
    if (hp_ratio < 0.3) heal_score += 40;   // 血量 < 30% 紧急加分
    if (hp_ratio < 0.2) heal_score += 30;   // 血量 < 20% 极危加分
    if (base_threat) heal_score += 20;       // 基地受威胁时自身生存更重要

    // 成本归一化到 0-100 (相对 300 金币)
    int heal_cost_score = remote_heal_cost * 100 / 300;

    // 触发条件: 脱战 + HP < 40% + 预算够 + 收益 > 成本
    if (hp_ratio < 0.4 && available >= remote_heal_cost && heal_score > heal_cost_score) {
      trig_hp = 1;
    }
  }

  // ══════════════════════════════════════════════════════
  // 3. 远程补弹决策 (P2 优化 2 补给周期 + 补充 2)
  // ══════════════════════════════════════════════════════
  //   远程补弹: 150 金币 / 100 发, 有 6s 延迟
  //   非远程(补给区): 100 金币 / 100 发 (allow_ammo_target 支付)
  //   补给区免费产出: 100 发/min, **可累积** (不去不会过期)
  //
  //   补给周期意识 (优化 2):
  //     规则: 每分钟 +100 发免费弹量, 未领取的可累积
  //     (例: 6 分钟没去, 第 6 分钟去一次直接拿 600 发)
  //     这意味着补给区的免费弹量是一笔"存款", 迟早都能拿到
  //     → 非紧急时优先省钱等物理领取, 减少远程购买(150金币/100发)
  //
  //   堡垒储备弹量公式 (补充 5):
  //     N = 100 + 2 * floor(Δ/15), N ≤ 500, Δ = base_max - base_hp
  int ammo_score = 0;
  if (disengaged && can_ammo) {
    int elapsed_s = 420 - remain_s;

    // 当前累积的免费弹量 = floor(elapsed_s / 60) * 100
    // (只要去补给区刷一次卡就能全部领取)
    // int accumulated_free = (elapsed_s / 60) * 100;  // 留作后续使用

    // 弹量紧迫度
    double ammo_ratio = (ammo_target > 0)
      ? static_cast<double>(ammo) / ammo_target : 1.0;

    // 堡垒储备弹量计算 (补充 5)
    int base_deficit = std::max(0, base_max - base_hp);
    int fortress_target = std::min(FORTRESS_AMMO_MAX,
      100 + 2 * (base_deficit / 15));

    // 远程补弹收益评分
    ammo_score = 0;
    if (ammo < ammo_low) ammo_score += 60;         // 弹量低于警戒线
    else if (ammo_ratio < 0.5) ammo_score += 30;   // 弹量低于 50%
    if (base_threat && ammo < 150) ammo_score += 30; // 基地受威胁时弹量重要
    if (fortress_ammo < fortress_target) ammo_score += 15; // 堡垒弹量不足

    // 补给区免费弹量累积感知:
    //   accumulated_free = floor(elapsed_s / 60) * 100
    //   这笔"存款"去补给区刷一次卡就能全部领取, 不会过期
    //   如果累积量 >= 200 (约 2 分钟没去) 且弹量还没到极低
    //   → 优先省钱去物理领取, 降低远程购买动力
    int accumulated_free = (elapsed_s / 60) * SUPPLY_TICK_FREE_AMMO;
    if (accumulated_free >= 200 && ammo >= ammo_low / 2) {
      ammo_score -= 30;  // 有较多免费弹量未领取, 降低远程购买动力
    }

    // 远程补弹成本归一化
    int ammo_cost_score = REMOTE_AMMO_COST * 100 / 300;

    // 触发: 脱战 + 预算够 + 收益 > 成本
    if (ammo_score > ammo_cost_score && available >= REMOTE_AMMO_COST) {
      trig_ammo = 1;
    }
  }

  // ══════════════════════════════════════════════════════
  // 4. 统一收益函数 — 远程回血 vs 远程补弹互斥选择 (P2 优化 4)
  // ══════════════════════════════════════════════════════
  //   同时满足两个条件时，只执行收益更高的一个
  //   (因为裁判系统对同一帧的远程操作有频率限制)
  if (trig_hp == 1 && trig_ammo == 1) {
    if (heal_score >= ammo_score) {
      trig_ammo = 0;  // 回血优先
    } else {
      trig_hp = 0;    // 补弹优先
    }
  }

  // ══════════════════════════════════════════════════════
  // 5. 允许弹丸配额 — 单调递增 (P2 补充 3)
  // ══════════════════════════════════════════════════════
  //   0x0120 bit[2:12] allow_ammo_target 必须单调递增
  //   每次增加 50 发, 需要 allow_ammo 支付 (非远程 100/100 发)
  //
  //   堡垒储备弹量也纳入考虑: 如果堡垒弹量充足则减少 allow 增加频率
  {
    int base_deficit = std::max(0, base_max - base_hp);
    int fortress_target = std::min(FORTRESS_AMMO_MAX,
      100 + 2 * (base_deficit / 15));

    bool need_more_ammo = (ammo < ammo_target) ||
      (fortress_ammo < fortress_target && base_threat);

    // 非远程配额的成本 = 100 金币 / 100 发 (补充 2)
    // 但 allow_ammo_target 本身只是"允许值"，实际消耗在增益点刷卡时扣除
    // 这里用金币预算控制节奏
    int allow_cost = NONREMOTE_AMMO_COST_PER_100;  // 每增加 100 发对应 100 金币

    if (need_more_ammo && available >= allow_cost) {
      int new_allow = allow_ammo + 50;
      // 单调递增保证: 只允许增加，不允许减少
      allow_ammo = std::max(allow_ammo, new_allow);
    }
    // 最终保证单调递增 (安全兜底)
    // allow_ammo 已从 allow_ammo_target_in 读取上一帧值，这里只做 >=
  }

  // ══════════════════════════════════════════════════════
  // 6. 大能量机关决策
  // ══════════════════════════════════════════════════════
  //   条件: 基地受威胁 + 比赛中期 (elapsed 120~300s → remain 120~300s)
  //   或比赛末尾 base HP 低于 50% 时也启用
  {
    int elapsed_s = 420 - remain_s;
    bool mid_game = (elapsed_s >= 120 && elapsed_s <= 300);
    bool late_critical = (remain_s < 120 && base_hp < base_max / 2);

    if (base_threat && (mid_game || late_critical)) {
      big_energy = 1;
    }
  }

  // ══════════════════════════════════════════════════════
  // 7. 上升沿去重: 通过监控 remote_count 变化来避免重复触发
  // ══════════════════════════════════════════════════════
  //   如果 remote_heal_count 或 remote_ammo_count 没有变化 (即上一次触发还没被执行)
  //   但我们又要触发新的 → 这是合理的 (裁判系统靠上升沿检测)
  //   所以这里只做简单的去重: 如果 count 比上次记录多了 → 说明上次已执行
  if (trig_hp == 1 && remote_heal_cnt == prev_remote_heal_count_ &&
      prev_remote_heal_count_ > 0)
  {
    // 上一次触发可能还在处理中 (6s 延迟), 本帧不重复触发
    // 但如果 count 未增长可能只是还没收到回执, 保守起见仍触发
    // (裁判系统自行做上升沿去重)
  }
  prev_remote_heal_count_ = remote_heal_cnt;
  prev_remote_ammo_count_ = remote_ammo_cnt;

  // ══════════════════════════════════════════════════════
  // 8. 输出
  // ══════════════════════════════════════════════════════
  setOutput("trigger_remote_ammo", trig_ammo);
  setOutput("trigger_remote_hp", trig_hp);
  setOutput("enable_big_energy", big_energy);
  setOutput("allow_ammo_target_out", allow_ammo);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::DecideEconomyCmdAction>("DecideEconomyCmd");
}
