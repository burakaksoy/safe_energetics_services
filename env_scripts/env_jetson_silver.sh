#!/usr/bin/env bash

export ROS_WS=/home/jetson_silver/catkin_ws_kinect
export ROS_MELODIC=/opt/ros/melodic
source $ROS_MELODIC/setup.bash
source $ROS_WS/devel/setup.bash
export PATH=$ROS_ROOT/bin:$PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROS_WS
# export ROS_MASTER_URI=http://192.168.1.100:11311/
# export ROS_IP=192.168.1.102
export DISPLAY=:0 # For Kinect remote launch

killall gnome-terminal-server
gnome-terminal --display=:0 --command "bash -c \"source ~/.bashrc; roslaunch azure_kinect_ros_driver kinect_rgbd.launch; exec bash\""
exec "$@"