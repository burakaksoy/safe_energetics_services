# First, Make the windows computer known to the Linux servers by following:
# https://mcilis.medium.com/how-to-setup-passwordless-ssh-connect-from-windows-to-linux-b84881454b6a

$HOSTS=@("192.168.1.140", "192.168.1.142", "192.168.1.85") # ethernet
$USERNAMES=@("jetson_silver", "jetson_blue","wgc" )
$PASSWORDS=@("1234", "1234","WGCat90%")

$COMMAND="    "

for ($i = 0; $i -le $HOSTS.Length-1; $i = $i + 1){
    Write-Output  "------------"
    Write-Output  $i
    Write-Output  $USERNAMES[$i]

    $COMMAND = $COMMAND + "wt -w 0 nt Powershell -NoExit -c {
        ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms=`"ssh-rsa`" -o ConnectTimeout=2 -l $($USERNAMES[$i]) $($HOSTS[$i])
    }
    "
}

# Write-Output $COMMAND
Invoke-Expression $COMMAND