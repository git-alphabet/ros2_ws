cmake_minimum_required(VERSION 3.5)
project(rm_bt_test_publishers)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_decision_interfaces REQUIRED)

add_executable(robot_status_publisher src/robot_status_publisher.cpp)
ament_target_dependencies(robot_status_publisher rclcpp rm_decision_interfaces)

add_executable(detect_respawn_checker src/detect_respawn_checker.cpp)
ament_target_dependencies(detect_respawn_checker rclcpp rm_decision_interfaces)

install(TARGETS
  robot_status_publisher
  detect_respawn_checker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
