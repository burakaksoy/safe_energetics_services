#!/bin/bash
HOSTS=("192.168.1.140") 
USERNAMES=("jetson_silver")
PASSWORDS=("1234")  

SCRIPTS=(
# KINECT
"source ~/.bashrc;
source ~/catkin_ws_kinect/src/safe_energetics_services/env_scripts/env_jetson_silver.sh;
roslaunch azure_kinect_ros_driver kinect_rgbd.launch;
/bin/bash" 

# LEVEL SENSOR SERVICE
"source ~/.bashrc;
source ~/catkin_ws_kinect/src/safe_energetics_services/env_scripts/env_jetson_silver.sh;
sleep 10s;
cd ~/catkin_ws_kinect/src/safe_energetics_services/src/level_sensor_service; 
python level_sensor_service.py;
/bin/bash")

# Title of tabs for each script on remote device
TITLES=("KINECT" "LEVEL SENSOR")

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
