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

#ifndef GXU2026_NAV_BRINGUP__PROCESS_UTILS_HPP_
#define GXU2026_NAV_BRINGUP__PROCESS_UTILS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace gxu2026_process
{

/// 运行模式枚举
enum class CleanupMode
{
  SIM,      ///< 仿真模式
  REALITY   ///< 实车模式
};

/// 清理残留进程
/**
 * 清理 FastDDS 共享内存和按模式匹配的残留进程。
 * 根据模式（sim/reality）清理不同的进程 pattern。
 *
 * \param mode 运行模式
 */
void cleanup_residual_processes(CleanupMode mode);

/// 保存地图到文件
/**
 * 将地图数据转换为 YAML + PGM 格式并保存到指定路径。
 * 文件名格式：map_<YYYYMMDD_HHMM>.yaml（北京时间）
 *
 * \param map 地图数据（nav_msgs::msg::OccupancyGrid）
 * \param path 保存路径（目录）
 * \param prefix 文件名前缀（可选）
 * \return 保存成功返回 true，否则返回 false
 */
bool save_map_to_file(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & map,
  const std::string & path,
  const std::string & prefix = "map");

/// 获取北京时间戳字符串
/**
 * 返回格式：YYYYMMDD_HHMM
 *
 * \return 时间戳字符串
 */
std::string get_beijing_timestamp();

/// 获取模式对应的保存路径
/**
 * 根据模式返回对应的保存路径。
 *
 * \param mode 运行模式
 * \param base_path 基础路径（默认为 "maps"）
 * \return 保存路径
 */
std::string get_save_path(CleanupMode mode, const std::string & base_path = "maps");

/// 获取模式对应的进程 pattern 列表
/**
 * 根据模式返回需要清理的进程 pattern。
 *
 * \param mode 运行模式
 * \return 进程 pattern 列表
 */
std::vector<std::string> get_process_patterns(CleanupMode mode);

}  // namespace gxu2026_process

#endif  // GXU2026_NAV_BRINGUP__PROCESS_UTILS_HPP_
