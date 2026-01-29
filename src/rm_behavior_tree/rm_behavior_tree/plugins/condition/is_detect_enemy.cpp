#include "rm_behavior_tree/plugins/condition/is_detect_enemy.hpp"

namespace rm_behavior_tree
{

IsDetectEnemyAction::IsDetectEnemyAction(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsDetectEnemyAction::detectEnemyStatus, this), config)
{
}

BT::NodeStatus IsDetectEnemyAction::detectEnemyStatus()
{
  // 关键修改1：获取输入端口的消息类型改为 RMUL
  auto msg = getInput<auto_aim_interfaces::msg::RMUL>("message");

  if (!msg) {
    std::cerr << "Missing required input [message]" << '\n';
    return BT::NodeStatus::FAILURE;
  }

  // 关键修改2：直接判断 RMUL.msg 中的 bool 字段 is_detect_enemy
  // true = 检测到敌人（返回SUCCESS），false = 未检测到敌人（返回FAILURE）
  if (msg->is_detect_enemy) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsDetectEnemyAction>("IsDetectEnemy");
}
