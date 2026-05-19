#!/bin/bash
NAME=uwtec-streamer
DIR=/home/uwtec/USV/usv_ws
cd $DIR
export PATH=/home/uwtec/.pixi/bin:$PATH
export ROS_LOG_DIR=/home/uwtec/USV/usv_ws/log # 매우 중요
exec pixi run ros2 run uwtec_cart streamer
