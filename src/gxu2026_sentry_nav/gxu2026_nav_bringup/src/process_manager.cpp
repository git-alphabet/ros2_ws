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

#include "gxu2026_nav_bringup/process_manager.hpp"

#include <chrono>
#include <csignal>
#include <memory>
#include <string>

namespace gxu2026_process
{

// 静态成员初始化
std::atomic<bool> ProcessManager::signal_received_{false};
ProcessManager * ProcessManager::instance_ = nullptr;

ProcessManager::ProcessManager(const rclcpp::NodeOptions & options)
: Node("process_manager", options)
{
  // 声明参数
  this->declare_parameter("mode", "reality");
  this->declare_parameter("cleanup_on_start", true);
  this->declare_parameter("auto_save_interval_sec", 60);
  this->declare_parameter("map_save_path", "maps");

  // 获取参数
  mode_ = this->get_parameter("mode").as_string();
  cleanup_on_start_ = this->get_parameter("cleanup_on_start").as_bool();
  auto_save_interval_sec_ = this->get_parameter("auto_save_interval_sec").as_int();
  map_save_path_ = this->get_parameter("map_save_path").as_string();

  // 确定运行模式
  if (mode_ == "sim") {
    cleanup_mode_ = CleanupMode::SIM;
  } else {
    cleanup_mode_ = CleanupMode::REALITY;
  }

  RCLCPP_INFO(
    this->get_logger(), "ProcessManager started: mode=%s, cleanup_on_start=%s, interval=%ds",
    mode_.c_str(), cleanup_on_start_ ? "true" : "false", auto_save_interval_sec_);

  // 启动时自动清理
  if (cleanup_on_start_) {
    RCLCPP_INFO(this->get_logger(), "Cleaning up residual processes...");
    cleanup_residual_processes(cleanup_mode_);
  }

  // 订阅 /map 话题
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(10).transient_local(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      latest_map_ = msg;
      RCLCPP_DEBUG(this->get_logger(), "Received map update");
    });

  // 创建定时器
  if (auto_save_interval_sec_ > 0) {
    auto_save_timer_ = this->create_wall_timer(
      std::chrono::seconds(auto_save_interval_sec_),
      [this]() { auto_save_map(); });
  }

  // 创建手动保存服务
  save_map_service_ = this->create_service<std_srvs::srv::Trigger>(
    "save_map",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      (void)request;
      handle_save_map_service(response);
    });

  // 创建 LifecycleManager 服务客户端
  lifecycle_client_ = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
    "lifecycle_manager/manage_nodes");

  // 创建 MapSaver 服务客户端
  map_saver_client_ = this->create_client<nav2_msgs::srv::SaveMap>(
    "map_saver/save_map");

  // 设置全局实例指针，用于信号处理
  instance_ = this;

  // 注册信号处理器
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  RCLCPP_INFO(this->get_logger(), "ProcessManager initialized successfully");
}

ProcessManager::~ProcessManager()
{
  RCLCPP_INFO(this->get_logger(), "ProcessManager shutting down, saving map...");
  save_map_on_exit();

  // 调用 LifecycleManager shutdown
  if (lifecycle_client_ && lifecycle_client_->service_is_ready()) {
    RCLCPP_INFO(this->get_logger(), "Calling LifecycleManager shutdown...");
    call_lifecycle_manager(nav2_msgs::srv::ManageLifecycleNodes::Request::SHUTDOWN);
  }

  instance_ = nullptr;
}

void ProcessManager::signal_handler(int signum)
{
  (void)signum;
  signal_received_.store(true);

  if (instance_) {
    RCLCPP_INFO(instance_->get_logger(), "Signal received, saving map before exit...");
    instance_->save_map_on_exit();
  }

  // 重新抛出信号，让 ROS 2 的信号处理器处理
  std::signal(signum, SIG_DFL);
  std::raise(signum);
}

bool ProcessManager::save_map_now()
{
  if (!latest_map_) {
    RCLCPP_WARN(this->get_logger(), "No map received yet, cannot save");
    return false;
  }

  std::string save_path = get_save_path(cleanup_mode_, map_save_path_);

  RCLCPP_INFO(this->get_logger(), "Saving map to %s", save_path.c_str());

  if (save_map_to_file(latest_map_, save_path, "map")) {
    RCLCPP_INFO(this->get_logger(), "Map saved successfully");
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to save map");
    return false;
  }
}

void ProcessManager::auto_save_map()
{
  if (!latest_map_) {
    RCLCPP_WARN(this->get_logger(), "No map received yet, skipping auto save");
    return;
  }

  std::string save_path = get_save_path(cleanup_mode_, map_save_path_);
  std::string timestamp = get_beijing_timestamp();
  std::string map_url = save_path + "/map_" + timestamp;

  RCLCPP_INFO(
    this->get_logger(), "Auto saving map to %s.yaml",
    map_url.c_str());

  // 优先使用 nav2 MapSaver 服务
  if (call_map_saver(map_url)) {
    RCLCPP_INFO(this->get_logger(), "Map saved via nav2 MapSaver service");
  } else {
    // 回退到手动保存
    RCLCPP_WARN(this->get_logger(), "Falling back to manual map save");
    if (save_map_to_file(latest_map_, save_path, "map")) {
      RCLCPP_INFO(this->get_logger(), "Map saved manually");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save map");
    }
  }
}

void ProcessManager::save_map_on_exit()
{
  if (!latest_map_) {
    RCLCPP_WARN(this->get_logger(), "No map received, skipping save on exit");
    return;
  }

  std::string save_path = get_save_path(cleanup_mode_, map_save_path_);
  std::string timestamp = get_beijing_timestamp();
  std::string map_url = save_path + "/map_" + timestamp;

  RCLCPP_INFO(this->get_logger(), "Saving map on exit to %s.yaml", map_url.c_str());

  // 优先使用 nav2 MapSaver 服务
  if (call_map_saver(map_url)) {
    RCLCPP_INFO(this->get_logger(), "Map saved via nav2 MapSaver service on exit");
  } else {
    // 回退到手动保存
    RCLCPP_WARN(this->get_logger(), "Falling back to manual map save on exit");
    if (save_map_to_file(latest_map_, save_path, "map")) {
      RCLCPP_INFO(this->get_logger(), "Map saved manually on exit");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save map on exit");
    }
  }
}

void ProcessManager::handle_save_map_service(
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!latest_map_) {
    response->success = false;
    response->message = "No map received yet";
    RCLCPP_WARN(this->get_logger(), "Manual save failed: no map received");
    return;
  }

  std::string save_path = get_save_path(cleanup_mode_, map_save_path_);

  RCLCPP_INFO(this->get_logger(), "Manual saving map to %s", save_path.c_str());

  if (save_map_to_file(latest_map_, save_path, "map")) {
    response->success = true;
    response->message = "Map saved successfully to " + save_path;
    RCLCPP_INFO(this->get_logger(), "Manual map save successful");
  } else {
    response->success = false;
    response->message = "Failed to save map";
    RCLCPP_ERROR(this->get_logger(), "Manual map save failed");
  }
}

bool ProcessManager::call_lifecycle_manager(uint8_t command)
{
  if (!lifecycle_client_->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "LifecycleManager service not available");
    return false;
  }

  auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
  request->command = command;

  auto future = lifecycle_client_->async_send_request(request);

  // 等待服务响应
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(5)) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "LifecycleManager command %d succeeded", command);
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "LifecycleManager command %d failed", command);
      return false;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "LifecycleManager service call timed out");
    return false;
  }
}

bool ProcessManager::call_map_saver(const std::string & map_url)
{
  if (!map_saver_client_->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "MapSaver service not available, falling back to manual save");
    return false;
  }

  auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
  request->map_topic = "/map";
  request->map_url = map_url;
  request->image_format = "pgm";
  request->map_mode = "trinary";
  request->free_thresh = 0.196;
  request->occupied_thresh = 0.65;

  auto future = map_saver_client_->async_send_request(request);

  // 等待服务响应
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(10)) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    if (response->result) {
      RCLCPP_INFO(this->get_logger(), "MapSaver service call succeeded: %s", map_url.c_str());
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "MapSaver service call failed");
      return false;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "MapSaver service call timed out");
    return false;
  }
}

}  // namespace gxu2026_process

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<gxu2026_process::ProcessManager>(rclcpp::NodeOptions());

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
