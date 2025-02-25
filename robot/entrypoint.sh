#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node &
sleep 5
python3 robot_server.py
