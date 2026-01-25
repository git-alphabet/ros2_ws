#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rm_decision_interfaces/msg/robot_status.hpp"

using namespace std::chrono_literals;

class DetectRespawnChecker : public rclcpp::Node
{
public:
  DetectRespawnChecker()
  : Node("detect_respawn_checker")
  {
    sub_ = this->create_subscription<rm_decision_interfaces::msg::RobotStatus>(
      "/robot_status", 10, std::bind(&DetectRespawnChecker::cb, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "DetectRespawnChecker started");
  }

private:
  void cb(const rm_decision_interfaces::msg::RobotStatus::SharedPtr msg)
  {
    const int MAX_HP = 400;
    const int RESPAWN_STABLE_FRAMES = 2;
    const uint64_t RESPAWN_LOCK_TIMEOUT = 5000;

    int hp = static_cast<int>(msg->current_hp);
    bool hp_is_valid = (hp >= 0 && hp <= MAX_HP);
    bool is_dead_now = hp_is_valid ? (hp == 0) : true;

    uint64_t now_ms = static_cast<uint64_t>(this->now().nanoseconds() / 1000000ULL);

    // Simpler stable-detection logic: track whether we've seen alive frames since last dead.
    if (is_dead_now) {
      alive_stable_frames_ = 0;
      seen_alive_since_dead_ = false;
    } else if (hp_is_valid) {
      alive_stable_frames_ = std::min(alive_stable_frames_ + 1, RESPAWN_STABLE_FRAMES);
    }

    bool lock_expired = (now_ms - last_respawn_ms_ > RESPAWN_LOCK_TIMEOUT);
    if (respawn_locked_ && !lock_expired) {
      respawn_locked_ = true;
    } else {
      respawn_locked_ = false;
    }
    bool respawn_edge = (alive_stable_frames_ >= RESPAWN_STABLE_FRAMES) && (hp > 0) && hp_is_valid && (!respawn_locked_) && (!seen_alive_since_dead_);
    RCLCPP_INFO(this->get_logger(), "hp=%d is_dead=%d alive_frames=%d locked=%d respawn_edge=%d seen_alive_since_dead=%d", hp, (int)is_dead_now, alive_stable_frames_, (int)respawn_locked_, (int)respawn_edge, (int)seen_alive_since_dead_);

    if (respawn_edge) {
      respawn_locked_ = true;
      last_respawn_ms_ = now_ms;
      RCLCPP_INFO(this->get_logger(), "Detected respawn! hp=%d, setting need_recovery=true, recovery_start_ms=%llu", hp, (unsigned long long)now_ms);
      seen_alive_since_dead_ = true;
    }

    was_dead_ = is_dead_now;
  }

  rclcpp::Subscription<rm_decision_interfaces::msg::RobotStatus>::SharedPtr sub_;
  int alive_stable_frames_ = 0;
  bool respawn_locked_ = false;
  uint64_t last_respawn_ms_ = 0;
  bool seen_alive_since_dead_ = false;
  bool was_dead_ = true;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectRespawnChecker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
