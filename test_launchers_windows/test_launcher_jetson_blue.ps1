# First, Make the windows computer known to the Linux servers by following:
# https://mcilis.medium.com/how-to-setup-passwordless-ssh-connect-from-windows-to-linux-b84881454b6a

$HOSTS=@("192.168.1.142") # ethernet
$USERNAMES=@("jetson_blue")
$PASSWORDS=@("1234")

$SCRIPTS=@(
# KINECT    
"source ~/.bashrc;
source ~/catkin_ws_kinect/src/safe_energetics_services/env_scripts/env_jetson_blue.sh;
roslaunch azure_kinect_ros_driver kinect_rgbd.launch;
/bin/bash",

# LID CHECKER
"source ~/.bashrc;
source ~/catkin_ws_kinect/src/safe_energetics_services/env_scripts/env_jetson_blue.sh;
sleep 10s;
cd ~/catkin_ws_kinect/src/safe_energetics_services/src/lid_checker_service; 
python lid_checker_service.py;
/bin/bash",

# CUP DETECTOR
"source ~/.bashrc;
source ~/catkin_ws_kinect/src/safe_energetics_services/env_scripts/env_jetson_blue.sh;
sleep 10s;
cd ~/catkin_ws_kinect/src/safe_energetics_services/src/cup_detector_service; 
python cup_detector_service.py;
/bin/bash",

# PLACE DETECTOR
"source ~/.bashrc;
source ~/catkin_ws_kinect/src/safe_energetics_services/env_scripts/env_jetson_blue.sh;
sleep 10s;
cd ~/catkin_ws_kinect/src/safe_energetics_services/src/place_detector_service; 
python place_detector_service.py;
/bin/bash"
)

# Title of tabs for each script on remote device
$TITLES=@("KINECT", "LID CHECKER", "CUP DETECTOR", "PLACE DETECTOR") 

for ($i = 0; $i -le $HOSTS.Length-1; $i = $i + 1){
    Write-Output  "------------"
    # Write-Output  $i
    Write-Output  $USERNAMES[$i]

    for ($j = 0; $j -le $SCRIPTS.Length-1; $j = $j + 1){
        $COMMAND = "wt -w 0 nt Powershell -NoExit -c {
            ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms=`"ssh-rsa`" -o ConnectTimeout=2 $($USERNAMES[$i])@$($HOSTS[$i]) `"$($SCRIPTS[$j])`"
        }
        "

        # Write-Output $COMMAND
        Invoke-Expression $COMMAND
    }
}