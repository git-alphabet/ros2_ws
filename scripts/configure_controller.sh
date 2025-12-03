#!/bin/bash
source install/setup.bash

echo "配置控制器服务器..."

# 配置控制器服务器
ros2 service call /controller_server/change_state lifecycle_msgs/srv/ChangeState '{transition: {id: 1}}'

sleep 2

# 激活控制器服务器  
ros2 service call /controller_server/change_state lifecycle_msgs/srv/ChangeState '{transition: {id: 3}}'

echo "控制器配置完成！"

# 检查状态
ros2 service call /controller_server/get_state lifecycle_msgs/srv/GetState