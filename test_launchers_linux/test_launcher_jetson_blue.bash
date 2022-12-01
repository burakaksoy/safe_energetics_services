#!/bin/bash
HOSTS=("192.168.1.142")
USERNAMES=("jetson_blue")
PASSWORDS=("1234")  

SCRIPTS=(
# KINECT    
"source ~/.bashrc;
source ~/catkin_ws_kinect/src/safe_energetics_services/env_scripts/env_jetson_blue.sh;
roslaunch azure_kinect_ros_driver kinect_rgbd.launch;
/bin/bash" 

# LID CHECKER
"source ~/.bashrc;
source ~/catkin_ws_kinect/src/safe_energetics_services/env_scripts/env_jetson_blue.sh;
sleep 10s;
cd ~/catkin_ws_kinect/src/safe_energetics_services/src/lid_checker_service; 
python lid_checker_service.py;
/bin/bash"

# CUP DETECTOR
"source ~/.bashrc;
source ~/catkin_ws_kinect/src/safe_energetics_services/env_scripts/env_jetson_blue.sh;
sleep 10s;
cd ~/catkin_ws_kinect/src/safe_energetics_services/src/cup_detector_service; 
python cup_detector_service.py;
/bin/bash"

# PLACE DETECTOR
"source ~/.bashrc;
source ~/catkin_ws_kinect/src/safe_energetics_services/env_scripts/env_jetson_blue.sh;
sleep 10s;
cd ~/catkin_ws_kinect/src/safe_energetics_services/src/place_detector_service; 
python place_detector_service.py;
/bin/bash"
)

# Title of tabs for each script on remote device
TITLES=("KINECT" "LID CHECKER" "CUP DETECTOR" "PLACE DETECTOR") 

for i in ${!HOSTS[*]} ; do
    echo "------------"
    # echo ${HOSTS[i]}
    echo ${USERNAMES[i]}
    # echo ${PASSWORDS[i]}
    echo ${SCRIPTS[i]}
    ssh-keygen -f "$HOME/.ssh/known_hosts" -R ${HOSTS[i]}

    for j in ${!SCRIPTS[*]} ; do
        COMMAND="gnome-terminal"
     
        COMMAND+=' --tab --title='${TITLES[j]}' -e "sshpass -p '${PASSWORDS[i]}' ssh -X -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='"'ssh-rsa'"' -o ConnectTimeout=2  '${USERNAMES[i]}'@'${HOSTS[i]}' '${SCRIPTS[j]}'"'
        
        # echo $COMMAND
        eval $COMMAND
    done
done
