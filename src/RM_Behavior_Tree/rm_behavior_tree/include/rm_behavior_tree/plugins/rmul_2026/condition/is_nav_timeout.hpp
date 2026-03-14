#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NAV_TIMEOUT_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NAV_TIMEOUT_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{

/**
 * @brief 检测导航是否超时（长时间未到达目标点）
 *
 * 工作原理：
 *   - 内部维护一个计时器，首次 tick 时开始计时
 *   - elapsed < timeout_sec → FAILURE（未超时，正常导航中）
 *   - elapsed ≥ timeout_sec → SUCCESS（超时！需要触发 recovery）
 *   - 返回 SUCCESS 后自动重置计时器，为下一轮 recovery 周期做准备
 *
 * 设计说明：
 *   - 配合 Fallback 使用：IsAtNavGoal 失败 → IsNavTimeout 判断是否超时
 *   - 超时后父节点执行 MoveAround 等 recovery 动作
 *   - recovery 完成后计时器已重置，下一轮重新导航计时
 *
 * 端口：
 *   - timeout_sec (input): 超时阈值（秒），默认 30.0
 *   - nav_status (input):  是否已到达目标（bool），到达时重置计时器
 */
class IsNavTimeout : public BT::ConditionNode
{
public:
  IsNavTimeout(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("timeout_sec", 30.0, "navigation timeout in seconds"),
      BT::InputPort<bool>("nav_status", "{nav.is_at_goal}",
                          "if true (at goal), resets the timer")
    };
  }

  BT::NodeStatus tick() override;

private:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  TimePoint start_time_;
  bool timer_started_{false};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NAV_TIMEOUT_HPP_
