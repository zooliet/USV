#!/bin/bash
NAME=uwtec-twist-mux
DIR=/home/uwtec/USV/uwtec-cart
cd $DIR
export PATH=/home/uwtec/.pixi/bin:$PATH
export ROS_LOG_DIR=/home/uwtec/USV/logs # 매우 중요
exec pixi run ros2 launch uwtec_navigation joystick_and_twist_mux.launch.py
