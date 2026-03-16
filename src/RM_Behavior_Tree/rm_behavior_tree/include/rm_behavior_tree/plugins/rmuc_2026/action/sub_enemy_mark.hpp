#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_ENEMY_MARK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_ENEMY_MARK_HPP_

#include <string>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/rmuc_enemy_mark.hpp"

namespace rm_behavior_tree
{
/// 订阅 0x020C 敌方易伤/己方标识, 写入黑板 {enemy_mark}
class RmucSubEnemyMarkAction
  : public BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUCEnemyMark>
{
public:
  RmucSubEnemyMarkAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<rm_decision_interfaces::msg::RMUCEnemyMark>("enemy_mark")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::RMUCEnemyMark> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif
