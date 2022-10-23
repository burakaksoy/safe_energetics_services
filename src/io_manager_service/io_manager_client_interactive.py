#Simple example Robot Raconteur RobotRequest client
#This program ...
from RobotRaconteur.Client import *

import time
import sys

#Main program
def main():

    url='rr+tcp://localhost:9005/?service=IOManager'
    # url='rr+tcp://192.168.1.142:9005/?service=IOManager'
    # url='rr+tcp://192.168.55.11:9005/?service=IOManager'

    RRN.RequestTimeout = 60 # seconds

    #Start up Robot Raconteur and connect, standard by this point    
    c = RRN.ConnectService(url)
    while True:
        inp = get_input()
        print('c.' + str(inp))
        eval('c.' + str(inp))



def get_input():
    print("Available commands:")
    print("openGripper()")
    print("isGripperOpen()")
    print("closeGripper()")
    print("isGripperClosed()")
    print("openCabinetRoboDoor()")
    print("isCabinetRoboDoorOpen()")
    print("closeCabinetRoboDoor()")
    print("isCabinetRoboDoorClosed()")
    print("isCabinetOpDoorClosed()")
    print("openHopperLid()")
    print("isHopperLidOpen()")
    print("closeHopperLid()")
    print("isHopperLidClosed()")
    print("isHopperPressureGood()")
    print("lockGate()")
    print("isGateLocked()")
    print("unlockGate()")
    print("isGateClosed()")
    print("isEmergency()")
    if (sys.version_info > (3, 0)):
        inp = input("press enter an IO command ( e.g. openGripper() ):\n")
    else:
        inp = raw_input("press enter an IO command ( e.g. openGripper() ):")

    return inp

if __name__ == '__main__':
    main()