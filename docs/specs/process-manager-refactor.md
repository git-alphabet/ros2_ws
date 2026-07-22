# 进程管理 C++ 包重构规格说明

## Problem Statement

当前进程管理逻辑完全在 Python 脚本 (`scripts/launch_wrapper.py`, 1062行) 中实现，存在以下问题：

1. **职责混乱**：scripts 承担了环境变量设置、参数解析、进程清理、自动保存、生命周期管理等多种职责
2. **代码重复**：构建脚本 (`quick_build.sh` vs `complete_build.sh`) 有80+行重复代码；启动脚本 (`mapping.sh` vs `start_navigation.sh`) 有15+行重复代码
3. **scripts 变得臃肿**：违背了"scripts 是为了快速启动不臃肿"的初衷
4. **Python 性能限制**：进程管理、共享内存清理等操作在 Python 中效率较低
5. **难以测试**：逻辑耦合在脚本中，无法独立测试

## Solution

将进程管理逻辑重构为 C++ 包，让 scripts 回归"轻量入口"的定位：

- **scripts 只负责**：设置环境变量、调用 `ros2 launch`
- **C++ 包负责**：进程清理、自动保存地图、生命周期管理
- **launch 文件集成**：清理和管理逻辑在 launch 内部自动处理，无需手动调用

## User Stories

1. 作为开发者，我希望 `./scripts/nav.sh` 命令保持不变，只改变底层实现，以便现有工作流程不受影响
2. 作为开发者，我希望启动导航时自动清理残留进程，不需要手动执行清理命令
3. 作为开发者，我希望建图过程中地图自动定期保存，防止意外丢失地图数据
4. 作为开发者，我希望进程退出时自动保存当前地图，确保地图数据不丢失
5. 作为开发者，我希望仿真模式和实车模式使用不同的清理策略和保存路径，避免模式混淆
6. 作为开发者，我希望清理逻辑能够处理 FastDDS 共享内存残留，避免 DDS 初始化挂死
7. 作为开发者，我希望清理逻辑能够根据模式（sim/reality）匹配不同的进程 pattern
8. 作为开发者，我希望自动保存功能能够调用 nav2 的地图保存服务，保持与 nav2 的一致性
9. 作为开发者，我希望 C++ 包能够调用 nav2 LifecycleManager 服务，管理导航节点的生命周期
10. 作为开发者，我希望清理逻辑和管理逻辑分离，职责清晰，便于维护和测试
11. 作为开发者，我希望 `process_manager` 节点能够在 launch 文件中配置，无需额外的启动脚本
12. 作为开发者，我希望提供手动保存地图的服务接口，支持用户主动触发保存
13. 作为开发者，我希望清理逻辑能够清理指定模式的残留进程，包括 Gazebo、rviz2、behavior_tree 等
14. 作为开发者，我希望自动保存的地图文件命名包含时间戳，便于地图版本管理
15. 作为开发者，我希望 C++ 包提供单元测试，确保清理和保存逻辑的正确性
16. 作为开发者，我希望构建脚本 (`build.sh`) 能够独立编译 C++ 包，无需编译整个工作空间
17. 作为开发者，我希望 C++ 包的配置通过 ROS 2 参数管理，支持运行时调整
18. 作为开发者，我希望清理逻辑能够处理进程组（PGID），确保子进程也被正确清理
19. 作为开发者，我希望自动保存间隔可通过参数配置，默认为60秒
20. 作为开发者，我希望 C++ 包的日志输出清晰，便于调试和问题排查

## Implementation Decisions

### 模块设计

#### 模块 1：`process_manager`（主可执行文件）

**类型**：ROS 2 节点（常驻后台）

**接口**：
- 参数：
  - `mode` (string): "reality" 或 "sim"
  - `cleanup_on_start` (bool): 启动时是否自动清理，默认 true
  - `auto_save_interval_sec` (int): 自动保存间隔（秒），默认 60
  - `map_save_path` (string): 地图保存根路径，默认 "maps"
- 服务：
  - `save_map` (std_srvs::Trigger): 手动保存地图
- 订阅：
  - `/map` (nav_msgs::msg::OccupancyGrid): 缓存最新地图

**职责**：
1. 启动时调用 `process_utils` 清理残留进程
2. 订阅 `/map` 话题并缓存最新地图
3. 定时器每 N 秒自动保存地图
4. 信号处理：SIGINT/SIGTERM 时保存地图
5. 调用 nav2 LifecycleManager 服务管理节点生命周期

#### 模块 2：`process_utils`（内部库）

**类型**：C++ 静态库

**接口**：
```cpp
namespace gxu2026_process {

enum class CleanupMode { SIM, REALITY };

// 清理残留进程
void cleanup_residual_processes(CleanupMode mode);

// 保存地图到文件
bool save_map_to_file(
    const nav_msgs::msg::OccupancyGrid::SharedPtr & map,
    const std::string & path,
    const std::string & prefix);

}  // namespace gxu2026_process
```

**职责**：
1. 清理 FastDDS 共享内存（`/dev/shm/fastrtps_*`）
2. 按模式匹配进程 pattern 并清理
3. 进程组管理（PGID）
4. 地图文件格式转换（YAML + PGM）

### 接缝设计

**外部接缝**（调用者需要知道的）：
- Launch 文件：`Node(executable='process_manager', parameters=[...])`
- 手动保存服务：`ros2 service call /process_manager/save_map`

**内部接缝**（模块内部可测试的）：
- `cleanup_residual_processes()`：可独立测试清理逻辑
- `save_map_to_file()`：可独立测试保存逻辑
- LifecycleManager 调用：可通过 mock 服务测试

### 集成方式

**launch 文件集成**：
```python
# rm_navigation_reality_launch.py
Node(
    package='gxu2026_nav_bringup',
    executable='process_manager',
    parameters=[{
        'mode': 'reality',
        'cleanup_on_start': True,
        'auto_save_interval_sec': 60,
        'map_save_path': 'maps/reality',
    }],
)
```

**scripts 简化**：
```bash
#!/bin/bash
# scripts/nav.sh
export NO_NEW_TERMINAL=1
export QT_FONT_DPI=192
exec ros2 launch gxu2026_nav_bringup rm_navigation_reality_launch.py "$@"
```

### 模式区分

| 模式 | 保存路径 | 清理的进程 pattern |
|------|---------|-------------------|
| `reality` | `maps/reality/` | rm_navigation_reality_launch, rviz2, rm_behavior_tree, robot_state_publisher, ... |
| `sim` | `maps/sim/` | bringup_sim, Gazebo, rm_navigation_simulation_launch, rviz2, nav2_container, ... |

### 自动保存策略

| 触发条件 | 行为 |
|---------|------|
| 定时器（每 N 秒） | 保存当前地图到 `maps/<mode>/map_<timestamp>.yaml` |
| SIGINT/SIGTERM | 保存当前地图，然后退出 |
| 手动服务调用 | 立即保存地图 |

## Testing Decisions

### 测试原则

1. **只测试外部行为**：通过接口测试模块功能，不测试内部实现细节
2. **独立测试**：每个模块可独立测试，不依赖其他模块
3. **Mock 依赖**：使用 mock 对象隔离外部依赖（如文件系统、ROS 服务）

### 测试模块

#### 1. `process_utils` 单元测试

**测试内容**：
- `cleanup_residual_processes()`：
  - 测试 FastDDS 共享内存清理
  - 测试按模式匹配进程 pattern
  - 测试进程组清理
- `save_map_to_file()`：
  - 测试地图文件格式转换
  - 测试文件命名（包含时间戳）
  - 测试错误处理（路径不存在、权限不足）

**测试方式**：
- 使用 Google Test 框架
- Mock 文件系统操作
- Mock 进程列表

#### 2. `process_manager` 集成测试

**测试内容**：
- 节点启动和参数加载
- 自动保存定时器触发
- 手动保存服务调用
- 信号处理（SIGINT/SIGTERM）

**测试方式**：
- 使用 ROS 2 测试框架（launch_testing）
- Mock nav2 LifecycleManager 服务
- 使用临时目录保存地图

### Prior Art

项目中已有的测试：
- `src/gxu2026_sentry_nav/gxu2026_nav_bringup/test/`：launch 文件测试
- 其他包的单元测试：使用 Google Test 框架

## Out of Scope

1. **话题频率监控（watchdog）**：根据用户决策，此功能移除
2. **nav2 LifecycleManager 深度集成**：仅调用服务，不实现完整的生命周期管理
3. **GUI 工具**：不提供图形界面
4. **远程进程管理**：仅管理本地进程
5. **历史遗留脚本清理**：`scripts/used/` 目录不在本次重构范围内

## Future Work

以下功能在当前版本中未实现，但计划在未来版本中添加：

### 1. LifecycleManager 集成

**User Story 9**: "作为开发者，我希望 C++ 包能够调用 nav2 LifecycleManager 服务，管理导航节点的生命周期"

**状态**: 未实现（当前版本）

**原因**: LifecycleManager 集成需要与 nav2 的节点生命周期管理深度集成，涉及多个服务调用和状态管理。当前版本专注于进程清理和自动保存功能，LifecycleManager 集成将在后续版本中实现。

**计划实现方式**:
- 订阅 nav2 LifecycleManager 的状态变化
- 在进程退出时调用 LifecycleManager 的 shutdown 服务
- 在进程重启时调用 LifecycleManager 的 cleanup 和 configure 服务

### 2. nav2 地图保存服务集成

**User Story 8**: "作为开发者，我希望自动保存功能能够调用 nav2 的地图保存服务，保持与 nav2 的一致性"

**状态**: 未实现（当前版本）

**原因**: 当前版本使用手动实现的 PGM+YAML 写入，虽然功能完整，但可能与 nav2 的默认参数不一致。未来版本将调用 nav2 的 MapSaver 服务，确保参数一致性。

**计划实现方式**:
- 调用 nav2 的 `/map_saver/save_map` 服务
- 使用 nav2 的默认参数（occupied_thresh, free_thresh, negate 等）
- 支持自定义参数覆盖

### 3. `build.sh` 脚本

**User Story 16**: "作为开发者，我希望构建脚本 (`build.sh`) 能够独立编译 C++ 包，无需编译整个工作空间"

**状态**: 未实现（当前版本）

**原因**: 当前版本使用 `colcon build --packages-select gxu2026_nav_bringup` 命令即可实现独立编译，无需额外脚本。未来版本将添加更便捷的构建脚本。

## Further Notes

### 构建方式

C++ 包使用 CMake 构建，集成到现有的 colcon 构建系统：

```bash
# 单独编译 C++ 包
colcon build --packages-select gxu2026_nav_bringup

# 或使用现有的构建脚本
./scripts/quick_build.sh
```

### 依赖

- ROS 2 Humble
- nav2_msgs（LifecycleManager 服务）
- nav_msgs（OccupancyGrid 消息）
- std_srvs（Trigger 服务）
- Google Test（单元测试）

### 文件结构

```
src/gxu2026_sentry_nav/gxu2026_nav_bringup/
├── CMakeLists.txt
├── include/
│   └── gxu2026_nav_bringup/
│       └── process_utils.hpp
├── src/
│   ├── process_manager.cpp
│   └── process_utils.cpp
├── test/
│   ├── test_process_utils.cpp
│   └── test_process_manager.launch.py
├── launch/
│   ├── rm_navigation_reality_launch.py
│   ├── rm_navigation_simulation_launch.py
│   └── ...
└── config/
    ├── reality/
    │   └── nav2_params.yaml
    └── simulation/
        └── nav2_params.yaml
```

### 迁移策略

1. 保留现有的 `scripts/launch_wrapper.py` 作为备份
2. 逐步迁移：先实现 `process_utils`，再实现 `process_manager`
3. 测试通过后，修改 scripts 调用方式
4. 验证仿真和实车模式都正常工作
5. 删除旧的 `launch_wrapper.py` 和相关脚本

---

**Spec 创建时间**：2026-07-22
**状态**：Ready for Agent
**标签**：refactor, cpp, process-management
