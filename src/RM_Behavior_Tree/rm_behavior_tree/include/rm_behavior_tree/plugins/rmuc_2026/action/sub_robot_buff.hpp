#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_ROBOT_BUFF_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_ROBOT_BUFF_HPP_

#include <string>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/rmuc_robot_buff.hpp"

namespace rm_behavior_tree
{
/// 订阅 0x0204 机器人实时增益, 写入黑板 {robot_buff}
class RmucSubRobotBuffAction
  : public BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUCRobotBuff>
{
public:
  RmucSubRobotBuffAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<rm_decision_interfaces::msg::RMUCRobotBuff>("robot_buff")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::RMUCRobotBuff> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif
