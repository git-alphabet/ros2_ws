#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rm_decision_interfaces/msg/robot_status.hpp"

using namespace std::chrono_literals;

class RobotStatusPublisher : public rclcpp::Node
{
public:
  RobotStatusPublisher()
  : Node("robot_status_publisher")
  {
    pub_ = this->create_publisher<rm_decision_interfaces::msg::RobotStatus>("/robot_status", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&RobotStatusPublisher::onTimer, this));
    start_time_ = this->now();
  }

private:
  void onTimer()
  {
    auto now = this->now();
    auto elapsed = now - start_time_;
    rm_decision_interfaces::msg::RobotStatus msg;

    // First 3 seconds: hp = 0 (dead). After that: hp = 100 (alive)
    if (elapsed.seconds() < 3.0) {
      msg.current_hp = 0;
    } else {
      msg.current_hp = 100;
    }

    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing RobotStatus current_hp=%d", msg.current_hp);
  }

  rclcpp::Publisher<rm_decision_interfaces::msg::RobotStatus>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotStatusPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
