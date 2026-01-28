cmake_minimum_required(VERSION 3.10)
project(rm_behavior_tree)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

set(PACKAGE_INCLUDE_DEPENDS
    rm_decision_interfaces
    rm_interfaces
    rclcpp
    std_msgs
    behaviortree_cpp
    behaviortree_ros2
)

# 定义宏函数 add_target_dependencies
function(add_target_dependencies target)
    target_include_directories(${target} PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
    ament_target_dependencies(${target} ${PACKAGE_INCLUDE_DEPENDS})
    target_compile_definitions(${target} PRIVATE BT_PLUGIN_EXPORT)
endfunction()

ament_auto_add_executable(rm_behavior_tree 
    src/rm_behavior_tree.cpp
)
ament_auto_add_executable(rm_behavior_tree_minimal
    src/rm_behavior_tree_minimal.cpp
)
# Controllers
ament_auto_add_library(decision_switch SHARED
    plugins/control/decision_switch.cpp
)
add_target_dependencies(decision_switch)

# Decorators
ament_auto_add_library(rate_controller SHARED 
    plugins/decorator/rate_controller.cpp
)
add_target_dependencies(rate_controller)

# Conditions
ament_auto_add_library(is_game_time SHARED 
    plugins/condition/is_game_time.cpp
)
add_target_dependencies(is_game_time)

ament_auto_add_library(is_status_ok SHARED 
    plugins/condition/is_status_ok.cpp
)
add_target_dependencies(is_status_ok)

ament_auto_add_library(is_attacked SHARED 
    plugins/condition/is_attacked.cpp
)
add_target_dependencies(is_attacked)

ament_auto_add_library(is_friend_ok SHARED 
    plugins/condition/is_friend_ok.cpp
)
add_target_dependencies(is_friend_ok)

ament_auto_add_library(is_outpost_ok SHARED 
    plugins/condition/is_outpost_ok.cpp
)
add_target_dependencies(is_outpost_ok)

## is_detect_enemy skipped: depends on external package 'auto_aim_interfaces' not available in this workspace


ament_auto_add_library(is_hp_above SHARED 
    plugins/condition/is_hp_above.cpp
)
add_target_dependencies(is_hp_above)

ament_auto_add_library(is_hp_below SHARED 
    plugins/condition/is_hp_below.cpp
)
add_target_dependencies(is_hp_below)

ament_auto_add_library(is_dead SHARED 
    plugins/condition/is_dead.cpp
)
add_target_dependencies(is_dead)

ament_auto_add_library(is_recovery_needed SHARED 
    plugins/condition/is_recovery_needed.cpp
)
add_target_dependencies(is_recovery_needed)

ament_auto_add_library(is_supply_card_detected SHARED 
    plugins/condition/is_supply_card_detected.cpp
)
add_target_dependencies(is_supply_card_detected)

ament_auto_add_library(is_within_scope SHARED 
    plugins/condition/is_within_scope.cpp
)
add_target_dependencies(is_within_scope)

# Actions
ament_auto_add_library(send_goal SHARED 
    plugins/action/send_goal.cpp
)
add_target_dependencies(send_goal)

ament_auto_add_library(sub_all_robot_hp SHARED 
    plugins/action/sub_all_robot_hp.cpp
)
add_target_dependencies(sub_all_robot_hp)

ament_auto_add_library(sub_robot_status SHARED 
    plugins/action/sub_robot_status.cpp
)
add_target_dependencies(sub_robot_status)

ament_auto_add_library(sub_game_status SHARED 
    plugins/action/sub_game_status.cpp
)
add_target_dependencies(sub_game_status)

## Skipping sub_armors: depends on external package `auto_aim_interfaces` which is
## not present in this workspace. If you add that package, uncomment the
## following lines to enable building `sub_armors`.
# ament_auto_add_library(sub_armors SHARED
#     plugins/action/sub_armors.cpp
# )
# add_target_dependencies(sub_armors)

ament_auto_add_library(sub_decision_num SHARED
    plugins/action/sub_decision_num.cpp
)
add_target_dependencies(sub_decision_num)

ament_auto_add_library(get_current_location SHARED 
    plugins/action/get_current_location.cpp
)
add_target_dependencies(get_current_location)

ament_auto_add_library(move_around SHARED 
    plugins/action/move_around.cpp
)
add_target_dependencies(move_around)

ament_auto_add_library(robot_control SHARED 
    plugins/action/robot_control.cpp
)
add_target_dependencies(robot_control)

ament_auto_add_library(print_message SHARED 
    plugins/action/print_message.cpp
)
add_target_dependencies(print_message)

# Sentry follower action plugin (was present in source but missing from build targets)
ament_auto_add_library(sentry_follower SHARED
    plugins/action/sentry_follower.cpp
)
add_target_dependencies(sentry_follower)



ament_auto_add_library(detect_respawn_and_set_recovery SHARED 
    plugins/action/detect_respawn_and_set_recovery.cpp
)
add_target_dependencies(detect_respawn_and_set_recovery)

ament_auto_add_library(sub_rfid_status SHARED 
    plugins/action/sub_rfid_status.cpp
)
add_target_dependencies(sub_rfid_status)

ament_auto_add_library(init_blackboard_config SHARED 
    plugins/action/init_blackboard_config.cpp
)
add_target_dependencies(init_blackboard_config)

ament_auto_add_library(wait_and_heal SHARED 
    plugins/action/wait_and_heal.cpp
)
add_target_dependencies(wait_and_heal)

ament_auto_add_library(clear_recovery_flag SHARED 
    plugins/action/clear_recovery_flag.cpp
)
add_target_dependencies(clear_recovery_flag)

ament_auto_add_library(set_nav_goal_from_config SHARED
    plugins/action/set_nav_goal_from_config.cpp
)
add_target_dependencies(set_nav_goal_from_config)

ament_auto_add_library(micro_search_supply_card SHARED
    plugins/action/micro_search_supply_card.cpp
)
add_target_dependencies(micro_search_supply_card)

ament_auto_add_library(navigate_to_goal SHARED 
    plugins/condition/navigate_to_goal.cpp
)
add_target_dependencies(navigate_to_goal)

ament_auto_add_library(sub_robot_position SHARED 
    plugins/action/sub_robot_position.cpp
)
add_target_dependencies(sub_robot_position)

ament_auto_add_library(init_search_timer_if_needed SHARED 
    plugins/action/init_search_timer_if_needed.cpp
)
add_target_dependencies(init_search_timer_if_needed)

ament_auto_add_library(not_arrived SHARED 
    plugins/condition/not_arrived.cpp
)
add_target_dependencies(not_arrived)

ament_auto_add_library(keep_running SHARED 
    plugins/action/keep_running.cpp
)
add_target_dependencies(keep_running)

ament_auto_add_library(cancel_nav_goal SHARED 
    plugins/action/cancel_nav_goal.cpp
)
add_target_dependencies(cancel_nav_goal)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)