#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__INIT_SENTRY_CONFIG_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__INIT_SENTRY_CONFIG_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 将所有静态配置参数输出到黑板，仅执行一次
class InitSentryConfigAction : public BT::SyncActionNode
{
public:
  InitSentryConfigAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::string>("topic_game_status", "/game_status"),
      BT::OutputPort<std::string>("topic_robot_status", "/robot_status"),
      BT::OutputPort<std::string>("topic_rfid_status", "/rfid_status"),
      BT::OutputPort<std::string>("topic_robot_pose", "/robot_position"),
      BT::OutputPort<std::string>("topic_radar_tracks", "/radar/enemy_tracks"),
      BT::OutputPort<double>("home_x", 0.0), BT::OutputPort<double>("home_y", 0.0),
      BT::OutputPort<double>("supply_x", 0.0), BT::OutputPort<double>("supply_y", 0.0),
      BT::OutputPort<double>("base_buff_x", 0.0), BT::OutputPort<double>("base_buff_y", 0.0),
      BT::OutputPort<double>("outpost_buff_x", 0.0), BT::OutputPort<double>("outpost_buff_y", 0.0),
      BT::OutputPort<double>("fortress_ally_x", 0.0), BT::OutputPort<double>("fortress_ally_y", 0.0),
      BT::OutputPort<double>("fortress_enemy_x", 0.0), BT::OutputPort<double>("fortress_enemy_y", 0.0),
      BT::OutputPort<double>("central_highland_x", 0.0), BT::OutputPort<double>("central_highland_y", 0.0),
      BT::OutputPort<double>("ladder_highland_x", 0.0), BT::OutputPort<double>("ladder_highland_y", 0.0),
      BT::OutputPort<double>("defend_anchor_x", 0.0), BT::OutputPort<double>("defend_anchor_y", 0.0),
      BT::OutputPort<double>("patrol_wpt_0_x", 0.0), BT::OutputPort<double>("patrol_wpt_0_y", 0.0),
      BT::OutputPort<double>("patrol_wpt_1_x", 0.0), BT::OutputPort<double>("patrol_wpt_1_y", 0.0),
      BT::OutputPort<double>("patrol_wpt_2_x", 0.0), BT::OutputPort<double>("patrol_wpt_2_y", 0.0),
      BT::OutputPort<double>("arrive_radius", 0.35),
      BT::OutputPort<int>("hp_critical", 80),
      BT::OutputPort<int>("hp_low", 180),
      BT::OutputPort<int>("hp_safe", 280),
      BT::OutputPort<int>("heat_high", 210),
      BT::OutputPort<int>("heat_critical", 245),
      BT::OutputPort<int>("ammo_low", 80),
      BT::OutputPort<int>("ammo_target", 300),
      BT::OutputPort<int>("base_deficit_for_fortress", 500),
      BT::OutputPort<double>("enemy_near_base_radius", 2.0),
      BT::OutputPort<int>("objective_hold_ms", 12000),
      BT::OutputPort<int>("combat_fire_burst_ms", 180),
      BT::OutputPort<int>("combat_fire_pause_ms", 120)};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
