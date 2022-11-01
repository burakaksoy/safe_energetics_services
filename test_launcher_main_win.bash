#!/usr/bin/env bash

# export SAFE=/home/burak/catkin_ws_kinect_olddd/src/safe_energetics_services
export SAFE=/mnt/c/Users/burak/safe_energetics_services

# killall gnome-terminal-server
# sleep 1s

# ROBOT REQUEST SERVICE
gnome-terminal --tab --title="SERV-ROB REQ" --command "bash -c \"cd $SAFE/src/robot_request_service; python3 robot_request_service.py; exec bash\""
sleep 3s 
# IO MANAGER SERVICE
gnome-terminal --tab --title="SERV-IO" --command "bash -c \"cd $SAFE/src/io_manager_service; python3 io_manager_service.py; exec bash\""
sleep 3s 

# INTERACTIVE ROBOT REQUEST CLIENT
gnome-terminal --tab --title="CLIENT-ROB REQ" --command "bash -c \"cd $SAFE/src/robot_request_service; python3 robot_request_client_interactive.py; exec bash\""
sleep 1s 

# INTERACTIVE IO CLIENT
gnome-terminal --tab --title="CLIENT-IO" --command "bash -c \"cd $SAFE/src/io_manager_service; python3 io_manager_client_interactive.py; exec bash\""

exec "$@"