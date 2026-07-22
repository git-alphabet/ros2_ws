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

#include "gxu2026_nav_bringup/process_utils.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <set>
#include <signal.h>
#include <string>
#include <unistd.h>
#include <vector>

namespace gxu2026_process
{

namespace fs = std::filesystem;

// 获取指定模式的进程 pattern 列表
std::vector<std::string> get_process_patterns(CleanupMode mode)
{
  if (mode == CleanupMode::SIM) {
    return {
      "bringup_sim\\.launch\\.py",
      "ruby.*ign|ign.*gazebo|gz-server|gz-gui",
      "rm_navigation_simulation_launch\\.py",
      "(^|/)rviz2(\\s|$)",
      "(^|/)rm_behavior_tree(\\s|$)",
      "component_container_isolated.*nav2_container"
    };
  } else {
    return {
      "rm_navigation_reality_launch\\.py",
      "(^|/)rviz2(\\s|$)",
      "(^|/)rm_behavior_tree(\\s|$)",
      "(^|/)joint_state_publisher(\\s|$)",
      "(^|/)robot_state_publisher(\\s|$)",
      "(^|/)auto_aim_yaw_joint_state_bridge(\\s|$)",
      "component_container_isolated.*nav2_container"
    };
  }
}

// 获取进程的 PGID
static pid_t get_pgid(pid_t pid)
{
  return getpgid(pid);
}

// 检查进程是否存在
static bool pid_exists(pid_t pid)
{
  return kill(pid, 0) == 0;
}

// 终止进程组
static bool kill_pgid(pid_t pgid, int sig)
{
  if (pgid <= 0) {
    return false;
  }

  if (killpg(pgid, sig) == -1) {
    // 进程组不存在或无权限
    return false;
  }
  return true;
}

// 检查进程组是否存在
static bool pgid_exists(pid_t pgid)
{
  if (pgid <= 0) {
    return false;
  }
  return killpg(pgid, 0) == 0;
}

// 清理指定 PGID 文件对应的进程组
static void cleanup_pgid_file(const std::string & pgid_file_path, const std::string & title)
{
  if (!fs::exists(pgid_file_path)) {
    return;
  }

  try {
    std::ifstream file(pgid_file_path);
    pid_t pgid;
    file >> pgid;

    if (pgid <= 0) {
      fs::remove(pgid_file_path);
      return;
    }

    std::cerr << "[process_cleanup] Killing " << title << " PGID=" << pgid
              << " via pgid file ..." << std::endl;

    // 先发送 SIGTERM，再发送 SIGKILL
    for (int sig : {SIGTERM, SIGKILL}) {
      if (!kill_pgid(pgid, sig)) {
        break;
      }

      usleep(800000);  // 0.8秒

      // 检查进程组是否还存在
      if (!pgid_exists(pgid)) {
        break;
      }
    }

    try {
      fs::remove(pgid_file_path);
    } catch (...) {
      // 忽略删除错误
    }
  } catch (const std::exception & e) {
    std::cerr << "[process_cleanup] Warning: failed to read pgid file " << pgid_file_path
              << ": " << e.what() << std::endl;
    try {
      fs::remove(pgid_file_path);
    } catch (...) {
      // 忽略删除错误
    }
  }
}

// 清理 FastDDS 共享内存
static void cleanup_fastdds_shm()
{
  std::vector<fs::path> shm_files;

  // 查找 /dev/shm/fastrtps_* 文件
  try {
    for (const auto & entry : fs::directory_iterator("/dev/shm")) {
      if (entry.path().filename().string().find("fastrtps_") == 0) {
        shm_files.push_back(entry.path());
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "[process_cleanup] Warning: failed to scan /dev/shm: " << e.what() << std::endl;
    return;
  }

  if (shm_files.empty()) {
    return;
  }

  std::cerr << "[process_cleanup] Found " << shm_files.size() << " FastDDS SHM files" << std::endl;

  // 找到持有这些文件的进程并终止
  std::set<pid_t> holders;
  for (const auto & shm_file : shm_files) {
    try {
      // 使用 lsof 查找持有文件的进程
      std::string cmd = "lsof " + shm_file.string() + " 2>/dev/null | awk 'NR>1{print $2}'";
      FILE * pipe = popen(cmd.c_str(), "r");
      if (pipe) {
        char buffer[128];
        while (fgets(buffer, sizeof(buffer), pipe)) {
          pid_t pid = atoi(buffer);
          if (pid > 0 && pid != getpid()) {
            holders.insert(pid);
          }
        }
        pclose(pipe);
      }
    } catch (...) {
      // 忽略错误
    }
  }

  // 终止持有进程
  if (!holders.empty()) {
    std::cerr << "[process_cleanup] Killing " << holders.size()
              << " FastDDS SHM holder processes" << std::endl;

    for (pid_t pid : holders) {
      if (!pid_exists(pid)) {
        continue;
      }

      pid_t pgid = get_pgid(pid);
      if (pgid > 0) {
        kill_pgid(pgid, SIGTERM);
      } else {
        kill(pid, SIGTERM);
      }
    }

    usleep(800000);  // 0.8秒

    for (pid_t pid : holders) {
      if (pid_exists(pid)) {
        pid_t pgid = get_pgid(pid);
        if (pgid > 0) {
          kill_pgid(pgid, SIGKILL);
        } else {
          kill(pid, SIGKILL);
        }
      }
    }
  }

  // 删除共享内存文件
  int cleaned = 0;
  int failed = 0;
  for (const auto & shm_file : shm_files) {
    try {
      fs::remove(shm_file);
      cleaned++;
    } catch (...) {
      failed++;
    }
  }

  std::cerr << "[process_cleanup] FastDDS SHM cleanup: total=" << shm_files.size()
            << " cleaned=" << cleaned << " failed=" << failed << std::endl;
}

// 按模式匹配进程 pattern 并清理
static void cleanup_by_patterns(CleanupMode mode)
{
  auto patterns = get_process_patterns(mode);

  for (const auto & pattern : patterns) {
    try {
      // 使用 pgrep 查找匹配的进程
      std::string cmd = "pgrep -f '" + pattern + "' 2>/dev/null";
      FILE * pipe = popen(cmd.c_str(), "r");
      if (!pipe) {
        continue;
      }

      std::vector<pid_t> pids;
      char buffer[128];
      while (fgets(buffer, sizeof(buffer), pipe)) {
        pid_t pid = atoi(buffer);
        if (pid > 0 && pid != getpid()) {
          pids.push_back(pid);
        }
      }
      pclose(pipe);

      if (pids.empty()) {
        continue;
      }

      std::cerr << "[process_cleanup] Killing " << pids.size()
                << " processes matching pattern: " << pattern << std::endl;

      // 先发送 SIGTERM
      for (pid_t pid : pids) {
        if (!pid_exists(pid)) {
          continue;
        }

        pid_t pgid = get_pgid(pid);
        if (pgid > 0) {
          kill_pgid(pgid, SIGTERM);
        } else {
          kill(pid, SIGTERM);
        }
      }

      usleep(800000);  // 0.8秒

      // 检查是否还有存活的进程，发送 SIGKILL
      for (pid_t pid : pids) {
        if (pid_exists(pid)) {
          pid_t pgid = get_pgid(pid);
          if (pgid > 0) {
            kill_pgid(pgid, SIGKILL);
          } else {
            kill(pid, SIGKILL);
          }
        }
      }
    } catch (const std::exception & e) {
      std::cerr << "[process_cleanup] Warning: failed to cleanup pattern "
                << pattern << ": " << e.what() << std::endl;
    }
  }
}

void cleanup_residual_processes(CleanupMode mode)
{
  std::cerr << "[process_cleanup] Starting cleanup for mode: "
            << (mode == CleanupMode::SIM ? "sim" : "reality") << std::endl;

  // 清理 PGID 文件对应的进程组
  std::string pgid_file = (mode == CleanupMode::SIM) ?
    "/tmp/ros2_nav_sim.pgid" : "/tmp/ros2_nav_reality.pgid";
  std::string title = (mode == CleanupMode::SIM) ? "sim" : "reality";
  cleanup_pgid_file(pgid_file, title);

  // 清理 FastDDS 共享内存
  cleanup_fastdds_shm();

  // 按模式匹配进程 pattern 并清理
  cleanup_by_patterns(mode);

  std::cerr << "[process_cleanup] Cleanup completed" << std::endl;
}

bool save_map_to_file(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & map,
  const std::string & path,
  const std::string & prefix)
{
  // TODO(future): 使用 nav2 的 MapSaver 服务代替手动实现，保持与 nav2 的一致性
  // 当前实现使用手动 PGM+YAML 写入，可能与 nav2 的默认参数不一致
  // 未来版本应调用 nav2 的 /map_saver/save_map 服务

  if (!map) {
    std::cerr << "[process_utils] Error: map is null" << std::endl;
    return false;
  }

  try {
    // 创建保存目录
    fs::path save_dir(path);
    if (!fs::exists(save_dir)) {
      fs::create_directories(save_dir);
    }

    // 生成文件名（包含时间戳）
    std::string timestamp = get_beijing_timestamp();
    std::string base_filename = prefix + "_" + timestamp;
    fs::path yaml_path = save_dir / (base_filename + ".yaml");
    fs::path pgm_path = save_dir / (base_filename + ".pgm");

    // 保存 PGM 文件
    std::ofstream pgm_file(pgm_path, std::ios::binary);
    if (!pgm_file.is_open()) {
      std::cerr << "[process_utils] Error: failed to open PGM file: " << pgm_path << std::endl;
      return false;
    }

    // PGM 文件头
    pgm_file << "P5\n";
    pgm_file << "# CREATOR: gxu2026_process_manager\n";
    pgm_file << map->info.width << " " << map->info.height << "\n";
    pgm_file << "255\n";

    // 写入地图数据
    for (size_t i = 0; i < map->data.size(); ++i) {
      uint8_t value;
      if (map->data[i] == -1) {
        value = 205;  // 未知区域（灰色）
      } else if (map->data[i] == 0) {
        value = 254;  // 可通行区域（白色）
      } else {
        value = 0;    // 障碍物（黑色）
      }
      pgm_file.put(value);
    }

    pgm_file.close();

    // 保存 YAML 文件
    std::ofstream yaml_file(yaml_path);
    if (!yaml_file.is_open()) {
      std::cerr << "[process_utils] Error: failed to open YAML file: " << yaml_path << std::endl;
      return false;
    }

    yaml_file << "image: " << base_filename << ".pgm\n";
    yaml_file << "resolution: " << map->info.resolution << "\n";
    yaml_file << "origin: [" << map->info.origin.position.x << ", "
              << map->info.origin.position.y << ", "
              << map->info.origin.position.z << "]\n";
    yaml_file << "negate: 0\n";
    yaml_file << "occupied_thresh: 0.65\n";
    yaml_file << "free_thresh: 0.196\n";

    yaml_file.close();

    std::cerr << "[process_utils] Map saved to: " << yaml_path << " / " << pgm_path << std::endl;
    return true;
  } catch (const std::exception & e) {
    std::cerr << "[process_utils] Error saving map: " << e.what() << std::endl;
    return false;
  }
}

std::string get_beijing_timestamp()
{
  // 获取北京时间（UTC+8）
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);

  // 转换为北京时间（UTC+8）
  time += 8 * 3600;  // 加 8小时

  // 使用 gmtime_r 代替 gmtime，确保线程安全
  struct tm tm_info;
  gmtime_r(&time, &tm_info);

  char buffer[16];
  strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M", &tm_info);

  return std::string(buffer);
}

std::string get_save_path(CleanupMode mode, const std::string & base_path)
{
  if (mode == CleanupMode::SIM) {
    return base_path + "/sim";
  } else {
    return base_path + "/reality";
  }
}

}  // namespace gxu2026_process
