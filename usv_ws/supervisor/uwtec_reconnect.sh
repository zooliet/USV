#!/bin/bash
NAME=uwtec-reconnect
DIR=/home/uwtec/USV/usv_ws/src/uwtec_cart/script
cd $DIR
export PATH=/home/uwtec/.pixi/bin:$PATH
export ROS_LOG_DIR=/home/uwtec/USV/usv_ws/log # 매우 중요
exec python uwtec_reconnect.py
