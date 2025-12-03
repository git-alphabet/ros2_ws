#ifndef RM_BEHAVIOR_TREE__CONDITION__IS_STATUS_OK_HPP_
#define RM_BEHAVIOR_TREE__CONDITION__IS_STATUS_OK_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/robot_status.hpp" // 假设消息类型和路径正确

namespace rm_behavior_tree
{

class IsStatusOKAction : public BT::ConditionNode
{
public:
  IsStatusOKAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    // 定义输入端口及其默认值
    return {
      BT::InputPort<std::shared_ptr<rm_decision_interfaces::msg::RobotStatus>>("message", "Robot status message"),
      BT::InputPort<int>("hp_threshold", 0, "Minimum acceptable robot HP (inclusive)"), // 默认0，总能满足
      BT::InputPort<int>("blue_outpost_hp_threshold", 0, "Minimum acceptable blue outpost HP (inclusive)"), // 默认0
      BT::InputPort<int>("red_outpost_hp_threshold", 0, "Minimum acceptable red outpost HP (inclusive)"), // 默认0
      BT::InputPort<int>("heat_threshold", 9999, "Maximum acceptable shooter heat (inclusive)") // 默认9999，几乎总满足
    };
  }

  BT::NodeStatus tick() override;

private:
  // 注意：SimpleConditionNode 的构造函数需要一个 tick 函数指针，
  // 但直接使用 tick() 更符合 BT::ConditionNode 的标准用法。
  // 我们将覆盖 tick() 而不是使用 SimpleConditionNode 的绑定方式。
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__CONDITION__IS_STATUS_OK_HPP_
