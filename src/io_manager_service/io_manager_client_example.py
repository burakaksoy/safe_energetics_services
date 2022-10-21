#Simple example Robot Raconteur CupDetector client
#This program ...
from RobotRaconteur.Client import *

import time

#Main program
def main():

    # url='rr+tcp://localhost:9005/?service=IOManager'
    # url='rr+tcp://192.168.1.142:9005/?service=IOManager'
    url='rr+tcp://192.168.55.11:9005/?service=IOManager'

    #Start up Robot Raconteur and connect, standard by this point    
    c = RRN.ConnectService(url)

    print("Attempting to open  the gripper")
    c.openGripper()
    time.sleep(1)
    print("is gripper open? Answer: " +  str(c.isGripperOpen()))
    print("is gripper closed? Answer: " +  str(c.isGripperClosed()))

    print("Attempting to close the gripper")
    c.closeGripper()
    time.sleep(1)
    print("is gripper open? Answer: " +  str(c.isGripperOpen()))
    print("is gripper closed? Answer: " +  str(c.isGripperClosed()))

    print("Attempting to open the cabinet Robo Door")
    c.openCabinetRoboDoor()
    time.sleep(1)
    print("is Cabinet Robo Door open? Answer: " +  str(c.isCabinetRoboDoorOpen()))
    print("is Cabinet robo door closed? Answer: " +  str(c.isCabinetRoboDoorClosed()))

    print("Attempting to close the cabinet Robo Door")
    c.closeCabinetRoboDoor()
    time.sleep(1)
    print("is Cabinet Robo Door open? Answer: " +  str(c.isCabinetRoboDoorOpen()))
    print("is Cabinet robo door closed? Answer: " +  str(c.isCabinetRoboDoorClosed()))

    print("is Cabinet OPERATOR door closed? Answer: " +  str(c.isCabinetOpDoorClosed()))

    print("Attempting to open the hopper Lid")
    c.openHopperLid()
    time.sleep(1)
    print("is Hopper lid open? Answer: " +  str(c.isHopperLidOpen()))
    print("is Hopper lid closed? Answer: " +  str(c.isHopperLidClosed()))

    print("Attempting to close the hopper Lid")
    c.closeHopperLid()
    time.sleep(1)
    print("is Hopper lid open? Answer: " +  str(c.isHopperLidOpen()))
    print("is Hopper lid closed? Answer: " +  str(c.isHopperLidClosed()))

    print("is Hopper Pressure good? Answer: " +  str(c.isHopperPressureGood()))

    print("Attempting to lock the gate")
    c.lockGate()
    time.sleep(1)
    print("is gate locked? Answer: " +  str(c.isGateLocked()))
    print("Attempting to unlock the gate")
    c.unlockGate()
    time.sleep(1)
    print("is gate Locked? Answer: " +  str(c.isGateLocked()))
    print("is gate Closed? Answer: " +  str(c.isGateClosed()))
    
    print("is ther Emergency? Answer: " +  str(c.isEmergency()))


if __name__ == '__main__':
    main()