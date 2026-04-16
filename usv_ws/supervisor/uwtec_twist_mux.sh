#!/bin/bash
NAME=uwtec-twist-mux
DIR=/home/uwtec/USV/usv_ws
cd $DIR
export PATH=/home/uwtec/.pixi/bin:$PATH
export ROS_LOG_DIR=/home/uwtec/USV/usv_ws/log # 매우 중요
exec pixi run ros2 launch uwtec_cart joystick_and_twist_mux.launch.py
