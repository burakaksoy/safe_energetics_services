#!/usr/bin/env bash

export SAFE=/home/burak/catkin_ws_kinect_olddd/src/safe_energetics_services
# killall gnome-terminal-server
# sleep 1s

# test cup detector
gnome-terminal --tab --title="test cup detector" --command "bash -c \"cd $SAFE/src/cup_detector_service; ls -l; exec bash\""

# test lid checker
gnome-terminal --tab --title="test lid checker" --command "bash -c \"cd $SAFE/src/lid_checker_service; ls -l; exec bash\""

# test place detector
gnome-terminal --tab --title="test place detector" --command "bash -c \"cd $SAFE/src/place_detector_service; ls -l; exec bash\""

exec "$@"