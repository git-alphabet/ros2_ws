#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_PROJECTILE_ALLOWANCE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_PROJECTILE_ALLOWANCE_HPP_

#include <string>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/rmuc_projectile_allowance.hpp"

namespace rm_behavior_tree
{
/// 订阅 0x0208 允许发弹量与金币, 写入黑板 {projectile_allowance}
class RmucSubProjectileAllowanceAction
  : public BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUCProjectileAllowance>
{
public:
  RmucSubProjectileAllowanceAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<rm_decision_interfaces::msg::RMUCProjectileAllowance>(
        "projectile_allowance")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::RMUCProjectileAllowance> & last_msg)
    override;
};
}  // namespace rm_behavior_tree

#endif
