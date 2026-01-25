#include "rm_behavior_tree/plugins/action/send_goal.hpp"

namespace rm_behavior_tree
{

SendGoalAction::SendGoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{
}

bool SendGoalAction::isSameGoal_(const geometry_msgs::msg::PoseStamped & a,
                                const geometry_msgs::msg::PoseStamped & b) const
{
  // Small tolerances to avoid spamming due to tiny floating noise.
  constexpr double kPosEps = 1e-3;
  constexpr double kOriEps = 1e-4;

  auto absd = [](double x) { return std::fabs(x); };

  // If any field is NaN/Inf, treat as "different" to force publish (safer behavior).
  const double vals_a[] = {
    a.pose.position.x, a.pose.position.y, a.pose.position.z,
    a.pose.orientation.x, a.pose.orientation.y, a.pose.orientation.z, a.pose.orientation.w
  };
  const double vals_b[] = {
    b.pose.position.x, b.pose.position.y, b.pose.position.z,
    b.pose.orientation.x, b.pose.orientation.y, b.pose.orientation.z, b.pose.orientation.w
  };
  for (size_t i = 0; i < sizeof(vals_a)/sizeof(vals_a[0]); ++i) {
    if (!isFinite_(vals_a[i]) || !isFinite_(vals_b[i])) {
      return false;
    }
  }

  if (absd(a.pose.position.x - b.pose.position.x) > kPosEps) return false;
  if (absd(a.pose.position.y - b.pose.position.y) > kPosEps) return false;
  if (absd(a.pose.position.z - b.pose.position.z) > kPosEps) return false;

  if (absd(a.pose.orientation.x - b.pose.orientation.x) > kOriEps) return false;
  if (absd(a.pose.orientation.y - b.pose.orientation.y) > kOriEps) return false;
  if (absd(a.pose.orientation.z - b.pose.orientation.z) > kOriEps) return false;
  if (absd(a.pose.orientation.w - b.pose.orientation.w) > kOriEps) return false;

  return true;
}

bool SendGoalAction::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  auto res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
  if (!res) {
    throw BT::RuntimeError("error reading port [goal_pose]:", res.error());
  }
  geometry_msgs::msg::PoseStamped goal = res.value();

  // Optional throttle: default 0 keeps old behavior (always publish).
  int min_interval_ms = 0;
  auto r_interval = getInput<int>("min_interval_ms");
  if (r_interval) {
    min_interval_ms = r_interval.value();
    if (min_interval_ms < 0) {
      min_interval_ms = 0;
    }
  }

  // Use SYSTEM_TIME to keep consistent with stored last_pub_time_
  auto now = rclcpp::Clock(RCL_SYSTEM_TIME).now();

  if (min_interval_ms > 0 && has_last_) {
    const bool same_goal = isSameGoal_(goal, last_goal_);
    if (same_goal) {
      const int64_t dt_ns = (now - last_pub_time_).nanoseconds();
      const int64_t min_dt_ns = static_cast<int64_t>(min_interval_ms) * 1000000LL;

      // If clock jumps backward OR interval not reached, skip publish this tick.
      if (dt_ns < 0 || dt_ns < min_dt_ns) {
        return false;
      }
    }
  }

  msg.header.stamp = now;
  msg.header.frame_id = "map";
  msg.pose.position.x = goal.pose.position.x;
  msg.pose.position.y = goal.pose.position.y;
  msg.pose.position.z = goal.pose.position.z;
  msg.pose.orientation.x = goal.pose.orientation.x;
  msg.pose.orientation.y = goal.pose.orientation.y;
  msg.pose.orientation.z = goal.pose.orientation.z;
  msg.pose.orientation.w = goal.pose.orientation.w;

  // clang-format off
  std::cout << "Goal: [ "
    << std::fixed << std::setprecision(1)
    << goal.pose.position.x << ", "
    << goal.pose.position.y << ", "
    << goal.pose.position.z << ", "
    << goal.pose.orientation.x << ", "
    << goal.pose.orientation.y << ", "
    << goal.pose.orientation.z << ", "
    << goal.pose.orientation.w << " ]\n";
  // clang-format on

  last_goal_ = goal;
  last_pub_time_ = now;
  has_last_ = true;

  return true;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SendGoalAction, "SendGoal");
