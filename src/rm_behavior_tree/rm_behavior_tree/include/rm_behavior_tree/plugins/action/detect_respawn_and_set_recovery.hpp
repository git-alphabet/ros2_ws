#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__DETECT_RESPAWN_AND_SET_RECOVERY_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__DETECT_RESPAWN_AND_SET_RECOVERY_HPP_

#include <cstdint>
#include <string>
#include <algorithm>  // 用于std::min

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/robot_status.hpp"

namespace rm_behavior_tree
{

class DetectRespawnAndSetRecoveryAction
: public BT::RosTopicSubNode<rm_decision_interfaces::msg::RobotStatus>
{
public:
  DetectRespawnAndSetRecoveryAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::InputPort<int>("hp_cur"),  // 保留hp_cur端口（兼容XML）
      // inout黑板端口
      BT::BidirectionalPort<bool>("was_dead"),
      BT::BidirectionalPort<bool>("need_recovery"),
      BT::BidirectionalPort<std::uint64_t>("recovery_start_ms"),
      BT::BidirectionalPort<std::uint64_t>("search_start_ms"),
      BT::BidirectionalPort<std::uint64_t>("heal_start_ms")
    };
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::RobotStatus> & last_msg) override;

private:
  // 1. 复活防抖配置：连续N帧存活才判定真复活（10Hz下2帧=200ms，适配竞赛）
  static constexpr int RESPAWN_STABLE_FRAMES = 2;
  // 2. 存活状态连续帧数缓存（防抖核心）
  int alive_stable_frames_ = 0;
  // 3. 复活触发锁：避免短时间重复触发
  bool respawn_locked_ = false;
  // 4. 锁超时时间（ms）：触发复活后5秒内不重复检测
  static constexpr std::uint64_t RESPAWN_LOCK_TIMEOUT = 5000;
  // 5. 上次触发复活的时间戳（用于锁超时判断）
  std::uint64_t last_respawn_ms_ = 0;
  // 6. 当前血量（成员变量，避免局部变量未定义问题）
  int current_hp_ = 0;
  // 7. 机器人最大血量
  static constexpr int MAX_HP = 400;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__DETECT_RESPAWN_AND_SET_RECOVERY_HPP_
