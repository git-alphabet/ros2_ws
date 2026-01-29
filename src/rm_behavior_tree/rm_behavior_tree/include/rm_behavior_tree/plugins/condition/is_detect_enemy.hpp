#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_

// 包含RMUL.msg编译后的头文件
#include "auto_aim_interfaces/msg/rmul.hpp"
// 包含Header对应的头文件（可选，RMUL.msg已包含，编译后会自动依赖）
#include "std_msgs/msg/header.hpp"
#include "behaviortree_cpp/condition_node.h"
// 新增：ROS2时间相关头文件，用于时间戳判断
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

/**
 * @brief condition节点，用于判断视野内是否存在有效敌人
 * @param[in] message 识别模块的检测结果（RMUL.msg），包含bool is_detect_enemy字段和时间戳header
 */
class IsDetectEnemyAction : public BT::SimpleConditionNode
{
public:
  IsDetectEnemyAction(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus detectEnemyStatus();

  static BT::PortsList providedPorts()
  {
    // 输入端口类型保持为 auto_aim_interfaces::msg::RMUL，兼容带时间戳的msg
    return {
      BT::InputPort<auto_aim_interfaces::msg::RMUL>("message")
    };
  }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_
