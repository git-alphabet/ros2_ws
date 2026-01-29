#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_

// 替换为 RMUL.msg 对应的头文件（ROS编译后自动生成）
#include "auto_aim_interfaces/msg/rmul.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{

/**
 * @brief condition节点，用于判断视野内是否存在有效敌人
 * @param[in] message 识别模块的检测结果（RMUL.msg），包含bool is_detect_enemy字段
 */
class IsDetectEnemyAction : public BT::SimpleConditionNode
{
public:
  IsDetectEnemyAction(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus detectEnemyStatus();

  static BT::PortsList providedPorts()
  {
    // 关键修改：输入端口类型替换为 auto_aim_interfaces::msg::RMUL
    return {
      BT::InputPort<auto_aim_interfaces::msg::RMUL>("message")
    };
  }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_
