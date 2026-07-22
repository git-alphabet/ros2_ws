# Copyright 2026 GXU2026 RoboMaster Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""进程管理器 launch 工具函数"""

import os
from pathlib import Path

from launch_ros.actions import Node


def find_maps_save_dir(bringup_dir: str) -> str:
    """查找项目根目录的 maps 目录

    Args:
        bringup_dir: bringup 包的目录路径

    Returns:
        maps 目录的路径
    """
    maps_save_dir = os.path.join(bringup_dir, "maps")
    ws_candidate = Path(bringup_dir)
    for _ in range(6):
        ws_candidate = ws_candidate.parent
        candidate = ws_candidate / "maps"
        if candidate.exists():
            return str(candidate)
    return maps_save_dir


def create_process_manager_node(
    bringup_dir: str,
    mode: str,
    cleanup_on_start: bool = True,
    auto_save_interval_sec: int = 60,
) -> Node:
    """创建 process_manager 节点

    Args:
        bringup_dir: bringup 包的目录路径
        mode: 运行模式，'sim' 或 'reality'
        cleanup_on_start: 启动时是否自动清理
        auto_save_interval_sec: 自动保存间隔（秒）

    Returns:
        process_manager 节点
    """
    maps_save_dir = find_maps_save_dir(bringup_dir)

    return Node(
        package="gxu2026_nav_bringup",
        executable="process_manager",
        name="process_manager",
        output="screen",
        parameters=[
            {
                "mode": mode,
                "cleanup_on_start": cleanup_on_start,
                "auto_save_interval_sec": auto_save_interval_sec,
                "map_save_path": maps_save_dir,
            }
        ],
    )
