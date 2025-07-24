#!/bin/bash

# 如果不是 root 用户，自动切换为 root 并重新执行脚本
if [ "$EUID" -ne 0 ]; then
  echo "切换到 root 运行..."
  exec sudo "$0" "$@"
fi

cd /root/dev_ws/nationale

# 设置 ROS 环境变量与路径
export CAM_TYPE=usb
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash

# 判断是否需要重新构建
if [ -f src/updated ]; then
  update_flag=$(cat src/updated)
  if [ "$update_flag" == "true" ]; then
    echo "检测到更新标志，开始构建..."
    rm -frv configs
    colcon build 
    if [ $? -ne 0 ]; then
      echo "构建失败，请检查错误信息。"
      sleep 10
      exit 1
    fi
    echo "false" > src/updated
  else
    echo "未检测到更新标志，跳过构建。"
  fi
else
  echo "提示: 未找到 'updated' 文件，默认跳过构建。"
fi

source /root/dev_ws/nationale/install/setup.bash

# 设置 Qt 平台
export QT_QPA_PLATFORM=xcb

# 启动程序
ros2 run nationale app_cuno_dl

bash
