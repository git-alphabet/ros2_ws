#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SENTRY_CMD_MUX_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SENTRY_CMD_MUX_HPP_

#include <string>
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "rm_decision_interfaces/msg/rmuc_sentry_cmd.hpp"

namespace rm_behavior_tree
{
/**
 * 将各决策节点输出的指令字段复用为 RMUC 消息发布到 /sentry_cmd
 *
 * P2 补充 3: 0x0120 单调递增约束
 *   - allow_ammo_target (bit[2:12]): 必须单调递增, 本节点做最终兜底
 *   - trigger_remote_ammo/hp (bit[13:16]): 由裁判系统上升沿检测, 本节点负责做帧间沿控制
 */
class RmucSentryCmdMuxAction : public BT::RosTopicPubNode<rm_decision_interfaces::msg::RMUCSentryCmd>
{
public:
  RmucSentryCmdMuxAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  bool setMessage(rm_decision_interfaces::msg::RMUCSentryCmd & msg) override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("posture", "3", "姿态 1=进攻 2=防御 3=移动"),
      BT::InputPort<int>("confirm_respawn", "0", "confirm_respawn"),
      BT::InputPort<int>("confirm_instant_respawn", "0", "confirm_instant_respawn"),
      BT::InputPort<int>("allow_ammo_target", "0", "allow_ammo_target"),
      BT::InputPort<int>("trigger_remote_ammo", "0", "trigger_remote_ammo"),
      BT::InputPort<int>("trigger_remote_hp", "0", "trigger_remote_hp"),
      BT::InputPort<int>("enable_big_energy", "0", "enable_big_energy"),
      BT::BidirectionalPort<std::string>("cmd_state", "", "内部状态跟踪")};
  }

private:
  // P2 补充 3: 单调递增保护
  uint16_t last_allow_ammo_ = 0;   // 上一帧发送的 allow_ammo_target
  bool last_trig_ammo_ = false;    // 上一帧的 trigger_remote_ammo (用于沿控制)
  bool last_trig_hp_ = false;      // 上一帧的 trigger_remote_hp (用于沿控制)
};
}  // namespace rm_behavior_tree

#endif
