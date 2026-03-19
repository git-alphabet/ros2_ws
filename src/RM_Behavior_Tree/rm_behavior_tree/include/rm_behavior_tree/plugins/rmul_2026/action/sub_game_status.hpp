#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GAME_STATUS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GAME_STATUS_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/rmul_rob.hpp"
#include <cstdint>

namespace rm_behavior_tree
{
class SubGameStatusAction : public BT::RosTopicSubNode<rm_decision_interfaces::msg::RMULRob>
{
public:
  SubGameStatusAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<rm_decision_interfaces::msg::RMULRob>("game_status"),
      BT::OutputPort<std::uint64_t>("now_ms", "{time.now_ms}", "Current ROS time in milliseconds")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::RMULRob> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GAME_STATUS_HPP_