#!/bin/bash

source /opt/ros/diamondback/setup.bash
export ROS_PACKAGE_PATH=~/dev/drh-robotics-ros/ros/ardros:$ROS_PACKAGE_PATH

roslaunch `rospack find ardros`/launch/slam.launch
#read -p "Press [Enter] key to end ..."
