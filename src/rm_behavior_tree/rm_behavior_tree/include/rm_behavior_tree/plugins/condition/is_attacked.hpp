#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ATTACKED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ATTACKED_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/rmul.hpp"

namespace rm_behavior_tree
{

/**
 * @brief condition节点，用于判断机器人是否被攻击掉血
 * @param[in] message 机器人状态话题id
 */
class IsAttackedAction : public BT::SimpleConditionNode
{
public:
  IsAttackedAction(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkRobotAttacked();

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::shared_ptr<rm_decision_interfaces::msg::RMUL>>("message")};
  }
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ATTACKED_HPP_