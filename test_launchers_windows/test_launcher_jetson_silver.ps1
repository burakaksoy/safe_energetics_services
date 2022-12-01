# First, Make the windows computer known to the Linux servers by following:
# https://mcilis.medium.com/how-to-setup-passwordless-ssh-connect-from-windows-to-linux-b84881454b6a

$HOSTS=@("192.168.1.140") # ethernet
$USERNAMES=@("jetson_silver")
$PASSWORDS=@("1234")

$SCRIPTS=@(
# KINECT
"source ~/.bashrc;
source ~/catkin_ws_kinect/src/safe_energetics_services/env_scripts/env_jetson_silver.sh;
roslaunch azure_kinect_ros_driver kinect_rgbd.launch;
/bin/bash",

# LEVEL SENSOR SERVICE
"source ~/.bashrc;
source ~/catkin_ws_kinect/src/safe_energetics_services/env_scripts/env_jetson_silver.sh;
sleep 10s;
cd ~/catkin_ws_kinect/src/safe_energetics_services/src/level_sensor_service; 
python level_sensor_service.py;
/bin/bash"
)

# $SCRIPTS=@(
# # KINECT
# "ls;
# ls -l;
# /bin/bash",

# # LEVEL SENSOR SERVICE
# "ls;
# ls -l;
# /bin/bash"
# )

# Title of tabs for each script on remote device
$TITLES=@("KINECT", "LEVEL SENSOR")

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