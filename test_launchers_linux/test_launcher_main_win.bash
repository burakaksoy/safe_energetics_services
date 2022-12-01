#!/bin/bash
HOSTS=("192.168.1.85")
USERNAMES=("wgc")
PASSWORDS=("WGCat90%")  

SCRIPTS=(
# Robot request Service    
"cd ~/safe_energetics_services/src/robot_request_service;
python.exe ./robot_request_service.py;
Powershell" 

# IO Service    
"Start-Sleep -Seconds 1;
cd ~/safe_energetics_services/src/io_manager_service;
python.exe ./io_manager_service.py;
Powershell" 

# Robot request Service    
"Start-Sleep -Seconds 3;
cd ~/safe_energetics_services/src/robot_request_service;
python.exe ./robot_request_client_interactive.py;
Powershell" 

# IO Service    
"Start-Sleep -Seconds 3;
cd ~/safe_energetics_services/src/io_manager_service;
python.exe ./io_manager_client_interactive.py;
Powershell" 
)

# Title of tabs for each script on remote device
TITLES=("Rob.Request Service" "IO Service" "Rob.Req Client" "IO Client" ) 

for i in ${!HOSTS[*]} ; do
    echo "------------"
    # echo ${HOSTS[i]}
    echo ${USERNAMES[i]}
    # echo ${PASSWORDS[i]}
    echo ${SCRIPTS[i]}
    ssh-keygen -f "$HOME/.ssh/known_hosts" -R ${HOSTS[i]}

    for j in ${!SCRIPTS[*]} ; do
        COMMAND="gnome-terminal"
     
        #
        COMMAND+=' --tab --title='${TITLES[j]}' -e "sshpass -p '${PASSWORDS[i]}' ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='"'ssh-rsa'"' -o ConnectTimeout=2  '${USERNAMES[i]}'@'${HOSTS[i]}' '${SCRIPTS[j]}'"'
        
        # echo $COMMAND
        eval $COMMAND
    done
done
