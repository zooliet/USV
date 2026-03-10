#!/bin/bash
NAME=uwtec-localizer
DIR=/home/uwtec/USV/uwtec-cart
cd $DIR
export PATH=/home/uwtec/.pixi/bin:$PATH
export ROS_LOG_DIR=/home/uwtec/USV/logs # 매우 중요
exec pixi run ros2 run uwtec_localization localizer -- --gnss-port /dev/ttyGNSS --gyro-port /dev/ttyGYRO
