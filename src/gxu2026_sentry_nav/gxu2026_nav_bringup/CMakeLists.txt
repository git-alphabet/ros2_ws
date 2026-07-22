cmake_minimum_required(VERSION 3.8)
project(gxu2026_nav_bringup)

# 依赖查找
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# 编译 process_utils 静态库
add_library(process_utils STATIC
  src/process_utils.cpp
)
target_include_directories(process_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(process_utils
  rclcpp
  std_srvs
  nav_msgs
  nav2_msgs
)

# 编译 process_manager 可执行文件
add_executable(process_manager
  src/process_manager.cpp
)
target_include_directories(process_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(process_manager
  process_utils
)
ament_target_dependencies(process_manager
  rclcpp
  std_srvs
  nav_msgs
  nav2_msgs
)

# 安装可执行文件和库
install(TARGETS process_utils process_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装 launch、config 等共享文件
install(DIRECTORY launch config map pcd behavior_trees rviz
  DESTINATION share/${PROJECT_NAME}
)

# 测试配置
if(BUILD_TESTING)
  # 启用测试覆盖率
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")

  # Lint 测试
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()

  # 查找 Google Test
  find_package(ament_cmake_gtest REQUIRED)

  # 单元测试：process_utils
  ament_add_gtest(test_process_utils
    test/unit/test_process_utils.cpp
  )
  target_include_directories(test_process_utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_process_utils
    process_utils
  )
  ament_target_dependencies(test_process_utils
    rclcpp
    std_srvs
    nav_msgs
  )

  # 集成测试：process_manager
  ament_add_gtest(test_process_manager
    test/unit/test_process_manager.cpp
  )
  target_include_directories(test_process_manager PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_process_manager
    process_utils
  )
  ament_target_dependencies(test_process_manager
    rclcpp
    std_srvs
    nav_msgs
    nav2_msgs
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  map
  pcd
  config
  behavior_trees
  rviz
)
