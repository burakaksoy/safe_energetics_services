#!/bin/bash
# HOSTS=("192.168.1.139" "192.168.1.141" "192.168.1.216") # wifi
HOSTS=("192.168.1.140" "192.168.1.142" "192.168.1.85") # ethernet
# HOSTS=("192.168.55.10" "192.168.55.11" "192.168.1.216") # abb ethernet
USERNAMES=("jetson_silver" "jetson_blue" "wgc")
PASSWORDS=("1234" "1234" "WGCat90%" )

SCRIPTS=("cd ~/catkin_ws_kinect/src/safe_energetics_services; 
          git reset --hard; git pull;"
          
          "cd ~/catkin_ws_kinect/src/safe_energetics_services; 
          git reset --hard; git pull;"
          
          "cd ~/safe_energetics_services; 
          git reset --hard; git pull;")
for i in ${!HOSTS[*]} ; do
    echo "------------"
    # echo ${HOSTS[i]}
    echo ${USERNAMES[i]}
    # echo ${PASSWORDS[i]}
    echo ${SCRIPTS[i]}
    ssh-keygen -f "$HOME/.ssh/known_hosts" -R ${HOSTS[i]}
    # sudo apt-get install sshpass
    sshpass -p ${PASSWORDS[i]} ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l ${USERNAMES[i]} ${HOSTS[i]} "${SCRIPTS[i]}"
    # ssh -o StrictHostKeyChecking=no -l ${USERNAMES[i]} ${HOSTS[i]} "${SCRIPTS[i]}"
done
