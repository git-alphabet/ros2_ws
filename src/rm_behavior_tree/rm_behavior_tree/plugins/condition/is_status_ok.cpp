#include "rm_behavior_tree/plugins/condition/is_status_ok.hpp"

namespace rm_behavior_tree
{

IsStatusOKAction::IsStatusOKAction(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus IsStatusOKAction::tick()
{
  // 获取输入参数
  auto msg = getInput<std::shared_ptr<rm_decision_interfaces::msg::RobotStatus>>("message");
  int hp_threshold = 0;
  int blue_outpost_hp_threshold = 0;
  int red_outpost_hp_threshold = 0;
  int heat_threshold = 9999;

  // 获取阈值，没有设置则使用默认值
  getInput("hp_threshold", hp_threshold);
  getInput("blue_outpost_hp_threshold", blue_outpost_hp_threshold);
  getInput("red_outpost_hp_threshold", red_outpost_hp_threshold);
  getInput("heat_threshold", heat_threshold);

  // 检查消息是否有效
  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  // 检查所有条件是否满足
  bool hp_ok = (*msg)->current_hp >= hp_threshold;
  bool blue_outpost_ok = (*msg)->blue_outpost_hp >= blue_outpost_hp_threshold;
  bool red_outpost_ok = (*msg)->red_outpost_hp >= red_outpost_hp_threshold;
  bool heat_ok = (*msg)->shooter_heat <= heat_threshold;

  // 所有条件都必须满足
  if (hp_ok && blue_outpost_ok && red_outpost_ok && heat_ok) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsStatusOKAction>("IsStatusOK");
}
