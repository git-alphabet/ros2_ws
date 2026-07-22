// Copyright 2026 GXU2026 RoboMaster Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GXU2026_NAV_BRINGUP__PROCESS_MANAGER_HPP_
#define GXU2026_NAV_BRINGUP__PROCESS_MANAGER_HPP_

#include <atomic>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav2_msgs/srv/save_map.hpp"

#include "gxu2026_nav_bringup/process_utils.hpp"

namespace gxu2026_process
{

/// 进程管理器节点
/**
 * 常驻后台节点，负责：
 * 1. 启动时自动清理残留进程
 * 2. 订阅 /map 话题并缓存最新地图
 * 3. 定时保存地图
 * 4. 信号处理：SIGINT/SIGTERM 时保存地图
 * 5. 提供手动保存服务接口
 */
class ProcessManager : public rclcpp::Node
{
public:
  explicit ProcessManager(const rclcpp::NodeOptions & options);

  ~ProcessManager();

  // 获取最新地图（用于测试）
  nav_msgs::msg::OccupancyGrid::SharedPtr get_latest_map() const { return latest_map_; }

  // 手动触发保存（用于测试）
  bool save_map_now();

  // LifecycleManager 服务调用
  bool call_lifecycle_manager(uint8_t command);

  // MapSaver 服务调用
  bool call_map_saver(const std::string & map_url);

private:
  void auto_save_map();
  void save_map_on_exit();
  void handle_save_map_service(
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // 信号处理函数
  static void signal_handler(int signum);

  // 参数
  std::string mode_;
  bool cleanup_on_start_;
  int auto_save_interval_sec_;
  std::string map_save_path_;
  CleanupMode cleanup_mode_;

  // ROS 2 接口
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr auto_save_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr lifecycle_client_;
  rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr map_saver_client_;

  // 地图缓存
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;

  // 全局信号标志
  static std::atomic<bool> signal_received_;
  static ProcessManager * instance_;
};

}  // namespace gxu2026_process

#endif  // GXU2026_NAV_BRINGUP__PROCESS_MANAGER_HPP_
