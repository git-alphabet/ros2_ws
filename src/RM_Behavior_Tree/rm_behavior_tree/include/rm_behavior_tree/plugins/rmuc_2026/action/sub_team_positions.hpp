#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_TEAM_POSITIONS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_TEAM_POSITIONS_HPP_

#include <string>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/rmuc_team_positions.hpp"

namespace rm_behavior_tree
{
/// 订阅 0x020B 己方队友位置, 写入黑板 {team_positions}
class RmucSubTeamPositionsAction
  : public BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUCTeamPositions>
{
public:
  RmucSubTeamPositionsAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<rm_decision_interfaces::msg::RMUCTeamPositions>("team_positions")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::RMUCTeamPositions> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif
