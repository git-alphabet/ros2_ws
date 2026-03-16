#include "rm_behavior_tree/plugins/rmuc_2026/action/sentry_cmd_mux.hpp"
#include <algorithm>

namespace rm_behavior_tree
{

RmucSentryCmdMuxAction::RmucSentryCmdMuxAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicPubNode<rm_decision_interfaces::msg::RMUCSentryCmd>(name, conf, params)
{
}

bool RmucSentryCmdMuxAction::setMessage(rm_decision_interfaces::msg::RMUCSentryCmd & msg)
{
  int posture = 3;
  int confirm_respawn = 0, confirm_instant = 0;
  int allow_ammo = 0, trig_ammo = 0, trig_hp = 0, big_energy = 0;

  getInput("posture", posture);
  getInput("confirm_respawn", confirm_respawn);
  getInput("confirm_instant_respawn", confirm_instant);
  getInput("allow_ammo_target", allow_ammo);
  getInput("trigger_remote_ammo", trig_ammo);
  getInput("trigger_remote_hp", trig_hp);
  getInput("enable_big_energy", big_energy);

  // ── P2 补充 3: allow_ammo_target 单调递增约束 ──
  // 0x0120 bit[2:12] 必须单调递增, 裁判系统会校验
  uint16_t safe_allow = static_cast<uint16_t>(std::max(0, allow_ammo));
  safe_allow = std::max(safe_allow, last_allow_ammo_);
  last_allow_ammo_ = safe_allow;

  // ── P2 补充 3: trigger_remote 上升沿控制 ──
  // 0x0120 bit[13:16] 远程兑换 count 必须递增 +1
  // 裁判系统靠检测 0→1 上升沿, 所以需要先发 0 再发 1
  // 如果上一帧已经是 1 且本帧还是 1 → 需要先间隔一个 0
  bool cur_trig_ammo = (trig_ammo != 0);
  bool cur_trig_hp = (trig_hp != 0);

  // 如果上帧已发 1, 本帧必须先发 0 以形成上升沿
  if (last_trig_ammo_ && cur_trig_ammo) {
    cur_trig_ammo = false;  // 本帧降为 0, 下帧再升
  }
  if (last_trig_hp_ && cur_trig_hp) {
    cur_trig_hp = false;
  }

  last_trig_ammo_ = cur_trig_ammo;
  last_trig_hp_ = cur_trig_hp;

  msg.cmd_posture = static_cast<uint8_t>(posture);
  msg.cmd_confirm_respawn = (confirm_respawn != 0);
  msg.cmd_confirm_instant_respawn = (confirm_instant != 0);
  msg.cmd_allow_ammo_target = safe_allow;
  msg.cmd_trigger_remote_ammo = cur_trig_ammo;
  msg.cmd_trigger_remote_hp = cur_trig_hp;
  msg.cmd_enable_big_energy = (big_energy != 0);

  return true;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucSentryCmdMuxAction, "SentryCmdMux");
