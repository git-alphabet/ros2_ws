#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_FIELD_STATUS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_FIELD_STATUS_HPP_

#include <string>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/rmuc_field_status.hpp"

namespace rm_behavior_tree
{
/// 订阅 0x0101 场地增益点状态, 写入黑板 {field_status}
class RmucSubFieldStatusAction
  : public BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUCFieldStatus>
{
public:
  RmucSubFieldStatusAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<rm_decision_interfaces::msg::RMUCFieldStatus>("field_status")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::RMUCFieldStatus> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif
