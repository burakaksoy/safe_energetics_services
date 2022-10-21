#!/bin/bash
# HOSTS=("192.168.1.140" "192.168.1.142" "192.168.1.216")
HOSTS=("192.168.55.10" "192.168.55.11" "192.168.1.216")
USERNAMES=("jetson_silver" "jetson_blue" "TODO")
PASSWORDS=("1234" "1234" "1234" )

SCRIPTS=("  
            source ~/catkin_ws_kinect/src/safe_energetics_services/env_scripts/env_jetson_silver.sh;
            "
          
        "   source ~/catkin_ws_kinect/src/safe_energetics_services/env_scripts/env_jetson_blue.sh;
            "
          
          "cd ~")
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
