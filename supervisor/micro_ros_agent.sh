#!/bin/bash
NAME=micro-ros-agent
DIR=/home/uwtec/USV/microros_ws
cd $DIR
PATH=/home/uwtec/.pixi/bin:$PATH
export ROS_LOG_DIR=/home/uwtec/USV/logs # 매우 중요
exec pixi run ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUROS
