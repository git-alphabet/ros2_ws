#include "rm_behavior_tree/plugins/rmuc_2026/action/decide_posture.hpp"
#include <algorithm>
#include <array>

namespace rm_behavior_tree
{

DecidePostureAction::DecidePostureAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus DecidePostureAction::tick()
{
  // ═══════════════════════════════════════════════
  // 0. 读取输入
  // ═══════════════════════════════════════════════
  int hp = 400, hp_max = 400, heat = 0, heat_high = 210, elapsed = 0;
  bool has_target = false, base_threat = false, disengaged = false;
  std::uint64_t now_ms = 0;
  std::string active_subtree;
  int ref_posture = 0;  // 裁判系统反馈 (补充4)
  int cool_val = 0, def_pct = 0, vuln_pct = 0;
  int ammo = 300;

  getInput("hp_cur", hp); // 当前血量。低血量(< 30%) → 强烈偏防御(+25)；高血量(> 70%) → 略偏进攻(+8)
  getInput("hp_max", hp_max); // 最大血量。与 hp_cur 算比值 hp_ratio = hp_cur / hp_max
  getInput("heat_cur", heat); // 当前枪口热量。热量快满(> 85%) → 偏防御冷却(+15)；热量低+有目标 → 偏进攻(+10)
  getInput("heat_high", heat_high); // 枪口热量上限。与 heat_cur 算比值 heat_ratio = heat_cur / heat_high
  getInput("has_target", has_target);
  getInput("base_threat", base_threat);
  getInput("is_disengaged", disengaged);
  getInput("stage_elapsed_time", elapsed);
  getInput("now_ms", now_ms);
  getInput("active_subtree", active_subtree);
  getInput("current_posture", ref_posture);
  getInput("buff_cool_value", cool_val);
  getInput("buff_defense_pct", def_pct);
  getInput("buff_vulnerability_pct", vuln_pct);
  getInput("ammo_allow", ammo);

  double hp_ratio = (hp_max > 0) ? static_cast<double>(hp) / hp_max : 1.0;
  double heat_ratio = (heat_high > 0) ? static_cast<double>(heat) / heat_high : 0.0;

  // ═══════════════════════════════════════════════
  // 补充 4: 裁判系统姿态同步
  // ═══════════════════════════════════════════════
  if (ref_posture >= 1 && ref_posture <= 3 && ref_posture != tracked_posture_) {
    // 裁判系统与内部不同步 → 修正
    tracked_posture_ = ref_posture;
    last_posture_ = ref_posture;
    // 重置衰减追踪 (以裁判系统为准, 保守重置)
    posture_since_ms_ = now_ms;
  }

  // ═══════════════════════════════════════════════
  // 1. 任务绑定基础分 (优化建议 1)
  // ═══════════════════════════════════════════════
  // scores[0]=进攻, scores[1]=防御, scores[2]=移动
  std::array<int, 3> scores = {0, 0, 0};

  // 任务默认映射 → 高权重 (+40)
  if (active_subtree == "BaseDefense" || active_subtree == "CriticalSurvival") {
    scores[1] += 40;  // 防御
  } else if (active_subtree == "EngageCombat") {
    scores[0] += 40;  // 进攻
  } else if (!active_subtree.empty()) {
    // ObjectivePlanner / PatrolAndScan / RespawnRecovery / WeaknessRecovery / SustainAndEconomy
    scores[2] += 40;  // 移动
  } else {
    // 无任务信息时 → 基于状态推断 (向后兼容)
    if (base_threat) scores[1] += 30;
    else if (has_target) scores[0] += 30;
    else scores[2] += 30;
  }

  // ═══════════════════════════════════════════════
  // 2. 状态修调
  // ═══════════════════════════════════════════════

  // ── 血量 ──
  if (hp_ratio < 0.3) {
    scores[1] += 25;  // 低血量 → 强烈偏好防御
    scores[0] -= 15;
  } else if (hp_ratio < 0.5) {
    scores[1] += 10;
  } else if (hp_ratio > 0.7) {
    scores[0] += 8;   // 高血量 → 略偏进攻
  }

  // ── 热量 ──
  if (heat_ratio > 0.85) {
    scores[1] += 15;  // 热量快满 → 防御冷却
    scores[0] -= 10;  // 进攻冷却虽高但继续打可能超限
  } else if (heat_ratio < 0.3 && has_target) {
    scores[0] += 10;  // 热量充裕+有目标 → 进攻
  }

  // ── 弹量 ──
  if (ammo <= 0) {
    scores[0] -= 20;  // 无弹 → 进攻无意义
    scores[2] += 10;  // 移动去补弹
  } else if (ammo < 50) {
    scores[0] -= 8;
  }

  // ── 有目标 ──
  if (has_target && hp_ratio > 0.5) {
    scores[0] += 12;
  }

  // ── 基地威胁 ──
  if (base_threat) {
    scores[1] += 20;
    scores[2] -= 10;
  }

  // ── 脱战 ──
  if (disengaged && !base_threat && !has_target) {
    scores[2] += 15;  // 脱战+安全 → 移动最佳
  }

  // ── 比赛后半段 ──
  if (elapsed > 300) {
    scores[1] += 5;  // 残局略偏防守
  }

  // ═══════════════════════════════════════════════
  // 3. 增益感知 (补充 2, 3)
  // ═══════════════════════════════════════════════

  // 冷却增益值 > 30 意味着有额外冷却加成 (堡垒/大能量机关)
  // 进攻姿态下冷却 ×3, 收益极高 → 偏好进攻
  if (cool_val > 30) {
    scores[0] += 20;  // 堡垒+进攻 = 315/s 冷却
  } else if (cool_val > 0) {
    scores[0] += 8;
  }

  // 防御增益% > 25 意味着有额外防御加成 (增益点)
  // 防御姿态叠加效果好 → 偏好防御
  if (def_pct > 25) {
    scores[1] += 12;
  }

  // 易伤% > 0 意味着当前处于负防御状态
  // 此时再叠加进攻/移动姿态的 25% 易伤会更危险
  if (vuln_pct > 0) {
    scores[0] -= 15;   // 进攻 25% 额外易伤 → 叠加太危险
    scores[2] -= 10;   // 移动 25% 额外易伤
    scores[1] += 20;   // 防御减免 → 抵消部分易伤
  }

  // ═══════════════════════════════════════════════
  // 4. 衰减规避 (补充 1)
  // ═══════════════════════════════════════════════
  if (now_ms > 0 && posture_since_ms_ > 0) {
    std::uint64_t duration = now_ms - posture_since_ms_;
    if (duration >= DEGRADE_WARN_MS) {
      // 当前姿态即将/已经衰减 → 大幅降低当前姿态评分
      int cur_idx = tracked_posture_ - 1;
      if (cur_idx >= 0 && cur_idx < 3) {
        scores[cur_idx] -= 30;  // 强制惩罚当前衰减姿态
      }
      // 进攻衰减最小(-33%冷却), 如果当前不是进攻, 可以考虑切进攻
      if (tracked_posture_ != 1) {
        scores[0] += 10;
      }
    }
  }

  // ═══════════════════════════════════════════════
  // 5. 选择最高分姿态
  // ═══════════════════════════════════════════════
  int best_idx = 0;
  for (int i = 1; i < 3; ++i) {
    if (scores[i] > scores[best_idx]) best_idx = i;
  }
  int desired = best_idx + 1;  // 1-indexed

  // ═══════════════════════════════════════════════
  // 6. 滞回过滤 (优化建议 2)
  // ═══════════════════════════════════════════════
  int output = last_posture_;

  if (desired != last_posture_) {
    // 6a. 5s 硬冷却检查
    bool cooldown_ok = (now_ms == 0) ||
      (now_ms - last_switch_ms_ >= SWITCH_COOLDOWN_MS);

    // 6b. 收益阈值检查: 新姿态评分须超过当前姿态 ≥ HYSTERESIS_THRESHOLD
    int cur_score = scores[last_posture_ - 1];
    int new_score = scores[desired - 1];
    bool threshold_ok = (new_score - cur_score >= HYSTERESIS_THRESHOLD);

    // 6c. 衰减紧急切换: 接近衰减时降低阈值要求
    bool degrade_urgent = false;
    if (now_ms > 0 && posture_since_ms_ > 0) {
      std::uint64_t duration = now_ms - posture_since_ms_;
      if (duration >= DEGRADE_WARN_MS && tracked_posture_ == last_posture_) {
        degrade_urgent = true;
      }
    }

    if (cooldown_ok && (threshold_ok || degrade_urgent)) {
      output = desired;
      last_switch_ms_ = now_ms;
    }
  }

  // ═══════════════════════════════════════════════
  // 7. 更新衰减追踪
  // ═══════════════════════════════════════════════
  if (output != tracked_posture_) {
    tracked_posture_ = output;
    posture_since_ms_ = now_ms;
  }

  last_posture_ = output;
  setOutput("posture_out", output);
  // Groot2 可视化: 输出各姿态实时评分
  setOutput("score_attack",  scores[0]);
  setOutput("score_defense", scores[1]);
  setOutput("score_move",    scores[2]);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::DecidePostureAction>("DecidePosture");
}
