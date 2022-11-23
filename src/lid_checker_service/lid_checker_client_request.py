#Simple example Robot Raconteur LidChecker client
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

        # url='rr+tcp://localhost:9004/?service=LidChecker'
        url='rr+tcp://192.168.1.142:9004/?service=LidChecker'
        # url='rr+tcp://192.168.55.11:9004/?service=LidChecker'

        #Start up Robot Raconteur and connect, standard by this point    
        c = RRN.ConnectService(url)

        if cmd == "isLidOff()" :
            lid_off = c.isLidOff()
            if lid_off:
                print("OFF")
            else:
                print("NOT OFF")

        if cmd == "isLidOn()" :
            lid_on = c.isLidOn()
            if lid_on:
                print("ON")
            else:
                print("NOT ON")

        if cmd == "isGraspedCorrectly()" :
            grasped_correctly = c.isGraspedCorrectly()
            if grasped_correctly:
                print("CORRECT")
            else:
                print("NOT CORRECT")

if __name__ == '__main__':
    main()