/**
 * @file rm_behavior_tree_rmuc.cpp
 * @brief RMUC 2026 哨兵行为树 —— 独立入口
 *
 * 与 RMUL 版本 (rm_behavior_tree.cpp) 完全分离：
 *   - 所有 RMUC.msg 通信统一到 /rmuc 话题（订阅 + 发布）
 *   - Groot2 使用不同端口 (1668) 以便同时调试
 *
 * 话题约定:
 *   /rmuc        — 所有 RMUC.msg 订阅者 & 发布者共用
 *   goal_pose    — SendGoal 发布 (通用 PoseStamped，与 RMUL 共享)
 */

#include "rm_behavior_tree/rm_behavior_tree.h"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;

  std::string bt_xml_path;
  auto node = std::make_shared<rclcpp::Node>("rmuc_behavior_tree");
  node->declare_parameter<std::string>(
    "style", "./config/rmuc_2026/rmuc_2026.xml");
  node->get_parameter_or<std::string>(
    "style", bt_xml_path, "./config/rmuc_2026/rmuc_2026.xml");

  std::cout << "Start RMUC_Behavior_Tree (2026)" << '\n';
  RCLCPP_INFO(node->get_logger(), "Load bt_xml: \e[1;42m %s \e[0m", bt_xml_path.c_str());

  // ═══════════════════════ ROS Node Params ═══════════════════════

  // 统一参数：所有 RMUC.msg 订阅者 / 发布者 / 混合节点均使用 /rmuc 话题
  BT::RosNodeParams params_rmuc;
  params_rmuc.nh = std::make_shared<rclcpp::Node>("rmuc_msg_io");
  params_rmuc.default_port_value = "/rmuc";

  // SendGoal (共享 RMUL 通用导航话题，PoseStamped 类型)
  BT::RosNodeParams params_send_goal;
  params_send_goal.nh = std::make_shared<rclcpp::Node>("send_goal");
  params_send_goal.default_port_value = "goal_pose";

  // ═══════════════════ 插件库列表 ════════════════════════════════

  // clang-format off

  // ── A. 所有 RMUC.msg ROS 节点插件 → RegisterRosNode + params_rmuc (/rmuc) ──
  //    包括订阅者、发布者、混合节点；全部走 /rmuc 话题
  const std::vector<std::string> rmuc_ros_plugin_libs = {
    // 订阅者
    "rmuc_sub_game_status",               // RmucSubGameStatus
    "rmuc_sub_robot_status",              // RmucSubRobotStatus
    "rmuc_sub_rfid_status",               // RmucSubRFIDStatus
    "rmuc_sub_robot_position",            // RmucSubRobotPosition
    "rmuc_sub_radar_tracks",              // SubRadarTracks
    "rmuc_detect_respawn_and_set_recovery", // RmucDetectRespawnAndSetRecovery
    "rmuc_wait_and_heal",                 // RmucWaitAndHeal
    // 发布者
    "rmuc_robot_control",                 // RmucRobotControl
    "rmuc_nav_control_cmd",               // RmucNavControlCmd
    "rmuc_sentry_cmd_mux",                // SentryCmdMux
    // 混合 (内部自行创建 publisher/subscriber)
    "rmuc_micro_search_supply_card",      // RmucMicroSearchSupplyCard
    "rmuc_is_supply_card_detected",       // RmucIsSupplyCardDetected
  };

  // ── B. 共享 RMUL CreateRosNodePlugin 库 → RegisterRosNode + params_rmuc ──
  const std::vector<std::string> shared_ros_plugin_libs = {
    "cancel_nav_goal",                    // CancelNavGoal
    "clear_recovery_flag",                // ClearRecoveryFlag
    "init_search_timer_if_needed",        // InitSearchTimerIfNeeded
    "is_recovery_needed",                 // IsRecoveryNeeded
  };

  // ── C. RMUC BT_REGISTER_NODES 纯 BT 插件（无需 ROS 参数） ──
  const std::vector<std::string> rmuc_bt_plugin_libs = {
    // ── 动作 ──
    "rmuc_init_sentry_config",            // InitSentryConfig
    "rmuc_init_cmd_state",                // InitCmdState
    "rmuc_decide_posture",                // DecidePosture
    "rmuc_decide_economy_cmd",            // DecideEconomyCmd
    "rmuc_decide_respawn_cmd",            // DecideRespawnCmd
    "rmuc_parse_sentry_blackboard",       // ParseSentryBlackboard
    "rmuc_select_safe_retreat_goal",      // SelectSafeRetreatGoal
    "rmuc_select_best_target",            // SelectBestTarget
    "rmuc_aim_at_target",                 // AimAtTarget
    "rmuc_fire_burst",                    // FireBurst
    "rmuc_hold_and_heal",                 // HoldAndHeal
    "rmuc_hold_for_supply_ammo_tick",     // HoldForSupplyAmmoTick
    "rmuc_select_nearest_resupply_station", // SelectNearestResupplyStation
    "rmuc_select_nearest_dispel_card",    // SelectNearestDispelCard
    "rmuc_select_objective",              // SelectObjective
    "rmuc_hold_objective",                // HoldObjective
    "rmuc_waypoint_patrol",              // WaypointPatrol
    // ── 条件 ──
    "rmuc_is_dead",                       // RmucIsDead
    "rmuc_is_game_time",                  // RmucIsGameTime
    "rmuc_is_hp_below",                   // RmucIsHPBelow
    "rmuc_is_at_nav_goal",                // RmucIsAtNavGoal
    "rmuc_is_at_goal",                    // IsAtGoal
    "rmuc_is_zone_card_detected",         // IsZoneCardDetected
    "rmuc_is_any_dispel_card_detected",   // IsAnyDispelCardDetected
    "rmuc_is_critical_state",             // IsCriticalState
    "rmuc_is_base_threatened",            // IsBaseThreatened
    "rmuc_has_valid_target",              // HasValidTarget
    "rmuc_is_combat_allowed",             // IsCombatAllowed
    "rmuc_is_fire_window_ok",             // IsFireWindowOk
    "rmuc_is_ammo_below",                // IsAmmoBelow
  };

  // ── D. 共享 RMUL BT_REGISTER_NODES 库 ──
  const std::vector<std::string> shared_bt_plugin_libs = {
    "rate_controller",                    // RateController
    "keep_running",                       // KeepRunning
    "move_around",                        // MoveAround (WeaknessRecovery 备用)
  };

  // clang-format on

  // ═══════════════════ 注册插件 ══════════════════════════════════

  // A. RMUC ROS 插件（订阅 + 发布 + 混合，全部 /rmuc）
  for (const auto & p : rmuc_ros_plugin_libs) {
    try {
      RegisterRosNode(factory, BT::SharedLibrary::getOSName(p), params_rmuc);
    } catch (const std::exception & e) {
      RCLCPP_WARN(node->get_logger(), "Could not load RMUC ROS plugin '%s': %s",
                  p.c_str(), e.what());
    }
  }

  // B. 共享 RMUL ROS 插件
  for (const auto & p : shared_ros_plugin_libs) {
    try {
      RegisterRosNode(factory, BT::SharedLibrary::getOSName(p), params_rmuc);
    } catch (const std::exception & e) {
      RCLCPP_WARN(node->get_logger(), "Could not load shared ROS plugin '%s': %s",
                  p.c_str(), e.what());
    }
  }

  // C. RMUC 纯 BT 插件
  for (const auto & p : rmuc_bt_plugin_libs) {
    try {
      factory.registerFromPlugin(BT::SharedLibrary::getOSName(p));
    } catch (const std::exception & e) {
      RCLCPP_WARN(node->get_logger(), "Could not load RMUC BT plugin '%s': %s",
                  p.c_str(), e.what());
    }
  }

  // D. 共享 RMUL BT 插件
  for (const auto & p : shared_bt_plugin_libs) {
    try {
      factory.registerFromPlugin(BT::SharedLibrary::getOSName(p));
    } catch (const std::exception & e) {
      RCLCPP_WARN(node->get_logger(), "Could not load shared BT plugin '%s': %s",
                  p.c_str(), e.what());
    }
  }

  // E. SendGoal (PoseStamped，非 RMUC.msg，单独注册)
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("send_goal"),
                  params_send_goal);

  // ═══════════════════ 创建并执行行为树 ═════════════════════════

  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(bt_xml_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create behavior tree from '%s': %s",
                 bt_xml_path.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Groot2Publisher 端口 1668（RMUL 用 1667，避免冲突）
  const unsigned port = 1668;
  BT::Groot2Publisher publisher(tree, port);

  while (rclcpp::ok()) {
    tree.tickWhileRunning(std::chrono::milliseconds(10));
  }

  rclcpp::shutdown();
  return 0;
}
