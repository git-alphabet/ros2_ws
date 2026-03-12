cmake_minimum_required(VERSION 3.10)
project(rm_decision_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # ── RMUL 拆分后的独立消息 ──
  "msg/RMUL/RMULRob.msg"          # 电控发布：比赛状态/血量/RFID/位姿
  "msg/RMUL/RMULNav.msg"          # 导航发布：导航指令/机器人控制/视觉/到达状态
  # ── RMUC 拆分后的独立消息 ──
  "msg/RMUC/RMUCGameStatus.msg"
  "msg/RMUC/RMUCRobotStatus.msg"
  "msg/RMUC/RMUCRFIDStatus.msg"
  "msg/RMUC/RMUCRobotPosition.msg"
  "msg/RMUC/RMUCEnemyTracks.msg"
  "msg/RMUC/RMUCSentryCmd.msg"
  "msg/RMUC/RMUCRobotControl.msg"
  "msg/RMUC/RMUCNavControlCmd.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
)

ament_package()