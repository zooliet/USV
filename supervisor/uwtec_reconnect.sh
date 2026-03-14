#!/bin/bash
NAME=uwtec-reconnect
DIR=/home/uwtec/USV/uwtec-cart/src/uwtec_agent/script
cd $DIR
export PATH=/home/uwtec/.pixi/bin:$PATH
export ROS_LOG_DIR=/home/uwtec/USV/logs # 매우 중요
# exec pixi run ros2 run uwtec_agent uwtec_agent
exec python uwtec_reconnect.py
