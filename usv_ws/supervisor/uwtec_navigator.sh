#!/bin/bash
NAME=uwtec-navigator
DIR=/home/uwtec/USV/usv_ws
cd $DIR
export PATH=/home/uwtec/.pixi/bin:$PATH
export ROS_LOG_DIR=/home/uwtec/USV/usv_ws/log # 매우 중요
exec pixi run ros2 run uwtec_cart navigator
# exec pixi run /home/uwtec/USV/usv_ws/.pixi/envs/default/bin/python /home/uwtec/USV/usv_ws/.pixi/envs/default/lib/uwtec_cart/navigator
