#include "rm_behavior_tree/plugins/rmuc_2026/action/select_objective.hpp"

namespace rm_behavior_tree
{

constexpr const char * SelectObjectiveAction::CANDIDATE_NAMES[];

SelectObjectiveAction::SelectObjectiveAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus SelectObjectiveAction::tick()
{
  // ═══════════════════════════════════════════════
  // 0. 读取输入
  // ═══════════════════════════════════════════════
  double px = 0, py = 0;
  int elapsed = 0, remain = 420;
  int hp = 400, hp_max = 400, ammo = 300, ammo_target = 300;
  int base_hp = 5000, base_max = 5000, deficit_thresh = 500;
  bool outpost_alive = true, base_threat = false;
  int f_central = 0, f_ladder = 0, f_fortress = 0, f_outpost = 0;
  bool f_base_buff = false;
  int fortress_ammo = 0;

  getInput("pose_x", px); getInput("pose_y", py);
  getInput("stage_elapsed_time", elapsed);
  getInput("stage_remain_time", remain);
  getInput("hp_cur", hp); getInput("hp_max", hp_max);
  getInput("ammo_allow", ammo); getInput("ammo_target", ammo_target);
  getInput("base_hp_cur", base_hp); getInput("base_hp_max", base_max);
  getInput("base_deficit_for_fortress", deficit_thresh);
  getInput("outpost_alive", outpost_alive);
  getInput("base_threat", base_threat);
  getInput("field_central_highland", f_central);
  getInput("field_ladder_highland", f_ladder);
  getInput("field_fortress", f_fortress);
  getInput("field_outpost_buff", f_outpost);
  getInput("field_base_buff", f_base_buff);
  getInput("fortress_ammo", fortress_ammo);

  // 读取各候选点坐标
  double coords[NUM_CANDIDATES][2] = {};
  getInput("central_highland_x", coords[CENTRAL_HIGHLAND][0]);
  getInput("central_highland_y", coords[CENTRAL_HIGHLAND][1]);
  getInput("ladder_highland_x", coords[TRAPEZOIDAL_HIGHLAND][0]);
  getInput("ladder_highland_y", coords[TRAPEZOIDAL_HIGHLAND][1]);
  getInput("base_buff_x", coords[BASE_BUFF][0]);
  getInput("base_buff_y", coords[BASE_BUFF][1]);
  getInput("outpost_buff_x", coords[OUTPOST_BUFF][0]);
  getInput("outpost_buff_y", coords[OUTPOST_BUFF][1]);
  getInput("fortress_ally_x", coords[FORTRESS_ALLY][0]);
  getInput("fortress_ally_y", coords[FORTRESS_ALLY][1]);
  getInput("fortress_enemy_x", coords[FORTRESS_ENEMY][0]);
  getInput("fortress_enemy_y", coords[FORTRESS_ENEMY][1]);
  getInput("buff_zone_x", coords[SUPPLY_ZONE][0]);
  getInput("buff_zone_y", coords[SUPPLY_ZONE][1]);
  getInput("defend_anchor_x", coords[DEFEND_ANCHOR][0]);
  getInput("defend_anchor_y", coords[DEFEND_ANCHOR][1]);

  // ═══════════════════════════════════════════════
  // 1. 派生变量
  // ═══════════════════════════════════════════════
  int base_deficit = std::max(0, base_max - base_hp);
  double hp_ratio = (hp_max > 0) ? static_cast<double>(hp) / hp_max : 1.0;
  double ammo_ratio = (ammo_target > 0) ? static_cast<double>(ammo) / ammo_target : 1.0;
  // 堡垒储备弹量上限: N = min(500, 100 + 2*floor(Δ/15))
  int fortress_target = std::min(500, 100 + 2 * (base_deficit / 15));

  // 阶段判定
  enum Phase { EARLY, MID, LATE };
  Phase phase = EARLY;
  if (elapsed >= 300) phase = LATE;
  else if (elapsed >= 120) phase = MID;

  // ═══════════════════════════════════════════════
  // 2. 计算各候选点评分 (优化建议 1: 多维评分)
  // ═══════════════════════════════════════════════
  //   score = survival + economy + defense + output - distance_penalty
  std::array<int, NUM_CANDIDATES> scores = {};

  for (int i = 0; i < NUM_CANDIDATES; ++i) {
    double dx = coords[i][0] - px;
    double dy = coords[i][1] - py;
    double dist = std::sqrt(dx * dx + dy * dy);

    // 基础: 距离惩罚 (越远越低, 每米 -5 分, 上限 -60)
    int dist_penalty = std::min(60, static_cast<int>(dist * 5));
    scores[i] = -dist_penalty;
  }

  // ── 中央高地 (补充 1: 25% 防御 + 补充 2: 排他占领) ──
  {
    int & s = scores[CENTRAL_HIGHLAND];
    s += 25;  // 基础防御收益
    if (f_central == 2) s += 40;  // 对方正在占 → 争夺价值高 (补充2)
    if (f_central == 1) s -= 30;  // 己方已占 → 不需要再去
    if (phase == EARLY) s += 20;  // 开局期加分 (优化建议 2)
    if (phase == LATE) s -= 10;   // 残局减分
  }

  // ── 梯形高地 (补充 1: 50% 防御, 仅己方可占) ──
  {
    int & s = scores[TRAPEZOIDAL_HIGHLAND];
    s += 40;  // 50% 防御, 高收益
    if (f_ladder == 1) s -= 30;  // 己方已占
    if (phase == EARLY) s += 15;
  }

  // ── 基地增益点 (补充 1: 50% 防御 + 兑换弹量 + 解除虚弱) ──
  {
    int & s = scores[BASE_BUFF];
    s += 35;  // 50% 防御 + 综合收益最高
    if (f_base_buff) s -= 20;  // 己方已占, 降低优先级
    if (hp_ratio < 0.5) s += 25;  // 低血量时回血价值高
    if (ammo_ratio < 0.4) s += 15;  // 低弹量时兑换弹量
    if (base_threat) s += 20;  // 基地受威胁时, 基地增益点也能防守
    if (phase == LATE) s += 15;  // 残局加分
  }

  // ── 前哨站增益点 (补充 1: 25% 防御 + 补充 3: 重建前哨站 + 补充 4: 前哨站→基地无敌) ──
  {
    int & s = scores[OUTPOST_BUFF];
    s += 20;  // 基础 25% 防御
    if (!outpost_alive && elapsed < 300) {
      // 前哨站被击毁 + 比赛 < 5 分钟 → 可能有重建机会 (补充3)
      // 重建前哨站 → 基地恢复无敌 (补充4) → 极高价值
      s += 60;
    }
    if (outpost_alive) {
      s -= 10;  // 前哨站存活时, 此点价值降低
    }
    if (f_outpost == 1) s -= 20;  // 己方已占
  }

  // ── 己方堡垒 (优化建议 3 + 补充 1: 50% 防御 + 冷却增益 + 储备弹量) ──
  {
    int & s = scores[FORTRESS_ALLY];
    s += 30;  // 基础防御收益
    // 基地掉血越多, 堡垒收益越高 (优化建议3)
    if (base_deficit > deficit_thresh) {
      s += std::min(40, base_deficit / 50);
    }
    // 堡垒储备弹量充足时, 价值更高 (在堡垒打免费弹)
    if (fortress_ammo > 50) s += 15;
    if (fortress_ammo > 200) s += 10;
    if (f_fortress == 1) s -= 15;  // 己方已占
    if (phase == MID) s += 15;  // 中盘期堡垒策略最佳
  }

  // ── 敌方堡垒 (补充 5: 占领 20s → 对方基地护甲展开, 但 100% 易伤) ──
  {
    int & s = scores[FORTRESS_ENEMY];
    s -= 20;  // 基础高风险
    // 条件: 对方前哨站已被击毁 + 比赛 > 3 分钟 (补充5)
    if (!outpost_alive && elapsed >= 180) {
      // 注: 这里 outpost_alive 是己方前哨站; 对方前哨站信息暂用同变量(TODO)
      // 进攻对方堡垒条件: 血量够承受 20s 易伤
      if (hp_ratio > 0.7 && hp > 300) {
        s += 50;  // 高血量时进攻收益大
      }
    }
    if (phase != LATE) s -= 10;  // 非残局时优先级低
    if (base_threat) s -= 30;  // 基地受威胁时不应远离
  }

  // ── 补给区 ──
  {
    int & s = scores[SUPPLY_ZONE];
    s += 10;  // 基础回血/免费弹量
    if (hp_ratio < 0.4) s += 30;
    if (ammo_ratio < 0.3) s += 25;
    // 补给区累积免费弹量: elapsed/60 * 100 (未领取时价值高)
    int accumulated_free = (elapsed / 60) * 100;
    if (accumulated_free >= 200) s += 15;
  }

  // ── 防御锚点 ──
  {
    int & s = scores[DEFEND_ANCHOR];
    if (base_threat) s += 80;  // 基地受威胁 → 最高优先回防
    if (!outpost_alive && base_hp < base_max / 2) s += 30;  // 前哨站没了+基地残血
    if (phase == LATE && base_threat) s += 20;  // 残局防守加分
    if (!base_threat) s -= 20;  // 基地安全时不需要守着
  }

  // ═══════════════════════════════════════════════
  // 3. 阶段化调整 (优化建议 2)
  // ═══════════════════════════════════════════════
  // 已在各候选点评分中体现 phase 加减分

  // ── 前哨站存活时可以更激进 (补充 4) ──
  if (outpost_alive) {
    // 基地无敌 → 不需要那么担心防守
    scores[DEFEND_ANCHOR] -= 15;
    scores[CENTRAL_HIGHLAND] += 10;
    scores[FORTRESS_ALLY] += 5;
  }

  // ═══════════════════════════════════════════════
  // 4. 选择最高分候选
  // ═══════════════════════════════════════════════
  int best_idx = 0;
  for (int i = 1; i < NUM_CANDIDATES; ++i) {
    if (scores[i] > scores[best_idx]) {
      best_idx = i;
    }
  }

  setOutput("goal_x", coords[best_idx][0]);
  setOutput("goal_y", coords[best_idx][1]);
  setOutput("objective_name", std::string(CANDIDATE_NAMES[best_idx]));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::SelectObjectiveAction>("SelectObjective");
}
