#!/bin/bash

source devel/setup.sh
ROS_MASTER_URI=http://alzemiro-dell:11311
rosrun motor_driver motor_driver_node
