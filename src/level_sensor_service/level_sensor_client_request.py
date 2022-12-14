#Simple example Robot Raconteur LevelSensor client
#This program ...
from RobotRaconteur.Client import *

import sys

#Main program
def main():
    argc = len(sys.argv)
    cmd = ""
    
    if argc == 2 :
        cmd = sys.argv[1]
 
    if len(cmd) > 0 :
        # url='rr+tcp://localhost:9001/?service=LevelSensor'
        url='rr+tcp://192.168.1.140:9001/?service=LevelSensor'
        # url='rr+tcp://192.168.55.10:9001/?service=LevelSensor'

        #Start up Robot Raconteur and connect, standard by this point    
        c = RRN.ConnectService(url)
        if ( c ):
        
            if cmd == "isLevelLow()":
                level_low = c.isLevelLow()
                if level_low:
                    print("TRUE")
                else:
                    print("FALSE")
            if cmd == "isLevelHigh()":
                level_high = c.isLevelHigh()
                if level_high:
                    print("TRUE")
                else:
                    print("FALSE")
        
        else:
            print("Open Failed " + url )
    else:
        print("No Command Sent" )

if __name__ == '__main__':
    main()