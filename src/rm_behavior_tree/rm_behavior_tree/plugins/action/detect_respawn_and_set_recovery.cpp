#include "rm_behavior_tree/plugins/action/detect_respawn_and_set_recovery.hpp"
#include "rclcpp/logging.hpp"  // 用于日志输出

namespace rm_behavior_tree
{
 
DetectRespawnAndSetRecoveryAction::DetectRespawnAndSetRecoveryAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::RobotStatus>(name, conf, params),
  alive_stable_frames_(0),
  respawn_locked_(false),
  last_respawn_ms_(0),
  current_hp_(0)
{
}

BT::NodeStatus DetectRespawnAndSetRecoveryAction::onTick(
  const std::shared_ptr<rm_decision_interfaces::msg::RobotStatus> & last_msg)
{
  // 1. 读取血量（优先黑板hp_cur，备用ROS消息）
  if (auto res = getInput<int>("hp_cur")) {
    current_hp_ = res.value();
  }
  // 备用方案：从ROS消息拿血量
  else if (last_msg) {
    current_hp_ = static_cast<int>(last_msg->current_hp);
  }
  // 没拿到血量，直接返回（保守策略）
  else {
    RCLCPP_WARN(node_->get_logger(), "未获取到机器人血量，跳过复活沿检测");
    return BT::NodeStatus::SUCCESS;
  }

  // 2. 读取历史死亡状态（was_dead）
  bool was_dead = false;
  if (auto res = getInput<bool>("was_dead")) {
    was_dead = res.value();
  }

  // 3. 读取当前恢复标志
  bool need_recovery = false;
  if (auto res = getInput<bool>("need_recovery")) {
    need_recovery = res.value();
  }

  // 4. 直接从ROS节点获取当前时间（毫秒级）
  std::uint64_t now_ms = static_cast<std::uint64_t>(node_->now().nanoseconds() / 1000000ULL);

  // 5. 优化1：血量合法性过滤（避免电控解析异常值）
  const bool hp_is_valid = (current_hp_ >= 0 && current_hp_ <= MAX_HP);
  // 只有血量合法时，才判断死亡状态；非法则默认判定为死亡
  const bool is_dead_now = hp_is_valid ? (current_hp_ == 0) : true;

  // 6. 优化2：连续帧防抖（核心抗干扰逻辑）
  if (!is_dead_now && hp_is_valid) {
    // 当前存活且血量合法：累计稳定帧数（最多到配置的帧数）
    alive_stable_frames_ = std::min(alive_stable_frames_ + 1, RESPAWN_STABLE_FRAMES);
  } else {
    // 当前死亡/血量异常：重置稳定帧数
    alive_stable_frames_ = 0;
  }

  // 7. 优化3：复活触发锁（避免短时间重复触发）
  bool lock_expired = (now_ms - last_respawn_ms_ > RESPAWN_LOCK_TIMEOUT);
  if (respawn_locked_ && !lock_expired) {
    // 锁未过期：保持锁定状态，跳过检测
    respawn_locked_ = true;
  } else {
    // 锁过期/未锁定：解锁
    respawn_locked_ = false;
  }

  // 8. 最终复活沿判断（竞赛级抗干扰）
  // 条件：历史死亡 + 连续N帧存活 + 血量合法且>0 + 未被锁定
  const bool respawn_edge = (was_dead && 
                            (alive_stable_frames_ >= RESPAWN_STABLE_FRAMES) && 
                            current_hp_ > 0 && 
                            hp_is_valid && 
                            !respawn_locked_);

  // 9. 触发复活沿：初始化恢复参数 + 加锁防重复
  if (respawn_edge) {
    need_recovery = true;
    setOutput("recovery_start_ms", now_ms);    // 记录恢复开始时间
    setOutput("search_start_ms", 0ULL);        // 重置搜卡时间
    setOutput("heal_start_ms", 0ULL);          // 重置回血时间
    
    // 加锁+记录触发时间（避免短时间重复触发）
    respawn_locked_ = true;
    last_respawn_ms_ = now_ms;

    // 竞赛调试：打印复活日志（方便赛场定位问题）
    RCLCPP_INFO(node_->get_logger(), 
          "检测到机器人复活沿！当前血量：%d，触发恢复模式", current_hp_);
  }

  // 10. 更新黑板状态（关键：同步最新状态）
  setOutput("was_dead", is_dead_now);          // 更新历史死亡状态
  setOutput("need_recovery", need_recovery);   // 更新恢复模式标志

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::DetectRespawnAndSetRecoveryAction, "DetectRespawnAndSetRecovery");
