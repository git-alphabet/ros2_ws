#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_

#include "behaviortree_cpp/condition_node.h"
// is_detect_enemy 直接从黑板读 bool，无需引入 msg 头文件

namespace rm_behavior_tree
{

/**
 * @brief 检测是否发现敌人的条件节点
 * 
 * 从RMUL消息中读取视觉检测字段is_detect_enemy
 * 如果检测到敌人返回SUCCESS,否则返回FAILURE
 * 
 * @param[in] message RMUL消息指针
 */
class IsDetectEnemyCondition : public BT::SimpleConditionNode
{
public:
  IsDetectEnemyCondition(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkEnemyDetected();

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("message")};
  }
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_