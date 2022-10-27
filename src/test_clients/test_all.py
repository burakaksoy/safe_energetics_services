#Simple example Robot Raconteur RobotRequest client
#This program ...
from RobotRaconteur.Client import *

import time
import sys

# !!!!!!!!!!!!!!!!!!!!VERY IMPORTANT WARNING!!!!!!!!!!!!!!!!!!!
# THIS SCRIPT DOES NOT CHECK FOR ANY IO SIGNAL!!!!!
# FOR EXAMPLE NO CHECKING FOR CABINET DOOR OPEN OR 
# GRASPING CORRECTLY
# BE READY TO STOP THE ROBOT WHEN RUNNING THIS SCRIPT!!!!!

#Main program
def main():
    url_io='rr+tcp://localhost:9005/?service=IOManager'
    url_rob_req='rr+tcp://localhost:9006/?service=RobotRequest'

    url_level='rr+tcp://192.168.55.10:9001/?service=LevelSensor'
    url_lid='rr+tcp://192.168.55.11:9004/?service=LidChecker'
    url_place='rr+tcp://192.168.55.11:9003/?service=PlaceDetector'
    url_cup='rr+tcp://192.168.55.11:9002/?service=CupDetector'

    RRN.RequestTimeout = 120 # seconds

    #Start up Robot Raconteur and connect, standard by this point    
    c_io = RRN.ConnectService(url_io)
    c_rob_req = RRN.ConnectService(url_rob_req)

    print("About to start: go2Home")
    get_input()
    c_rob_req.go2Home()

    print("Attempting to close the cabinet Robo Door")
    c_io.closeCabinetRoboDoor()
    time.sleep(3)
    print("is Cabinet Robo Door open? Answer: " +  str(c_io.isCabinetRoboDoorOpen()))
    print("is Cabinet robo door closed? Answer: " +  str(c_io.isCabinetRoboDoorClosed()))

    print("Attempting to close the hopper Lid")
    c_io.closeHopperLid()
    time.sleep(3)
    print("is Hopper lid open? Answer: " +  str(c_io.isHopperLidOpen()))
    print("is Hopper lid closed? Answer: " +  str(c_io.isHopperLidClosed()))

    print("Attempting to open the gripper")
    c_io.openGripper()
    time.sleep(2)
    print("is gripper open? Answer: " +  str(c_io.isGripperOpen()))
    print("is gripper closed? Answer: " +  str(c_io.isGripperClosed()))

    places_order = [1,0,3,2,5,4,7,6,9,8,11,10]
    cups_order = [0,1,2,3,4,5,6,7,8,9,10,11]

    for i in range(12):
        print("About to start: go2CabinetUpper")
        get_input()
        c_rob_req.go2CabinetUpper()

        print("Attempting to open the cabinet Robo Door")
        c_io.openCabinetRoboDoor()
        time.sleep(4)
        print("is Cabinet Robo Door open? Answer: " +  str(c_io.isCabinetRoboDoorOpen()))
        print("is Cabinet robo door closed? Answer: " +  str(c_io.isCabinetRoboDoorClosed()))

        # Check for available cups to pickup here
        # TODO

        print("About to start: go2Cup(" + str(cups_order[i]) + ")")
        get_input()
        c_rob_req.go2Cup(cups_order[i])

        print("Attempting to close the gripper")
        c_io.closeGripper()
        time.sleep(2)
        print("is gripper open? Answer: " +  str(c_io.isGripperOpen()))
        print("is gripper closed? Answer: " +  str(c_io.isGripperClosed()))

        print("About to start: go2CabinetUpper")
        get_input()
        c_rob_req.go2CabinetUpper()

        # Check for graspin here
        # TODO

        print("Attempting to close the cabinet Robo Door")
        c_io.closeCabinetRoboDoor()
        time.sleep(3)
        print("is Cabinet Robo Door open? Answer: " +  str(c_io.isCabinetRoboDoorOpen()))
        print("is Cabinet robo door closed? Answer: " +  str(c_io.isCabinetRoboDoorClosed()))

        print("About to start: go2LidRemover")
        get_input()
        c_rob_req.go2LidRemover()  

        print("About to start: RemoveCupLid")
        get_input()
        c_rob_req.RemoveCupLid()  

        # Check for cup lid removal here
        # TODO

        print("About to start: go2Hopper")
        get_input()
        c_rob_req.go2Hopper() 

        print("Attempting to open the hopper Lid")
        c_io.openHopperLid()
        time.sleep(3)
        print("is Hopper lid open? Answer: " +  str(c_io.isHopperLidOpen()))
        print("is Hopper lid closed? Answer: " +  str(c_io.isHopperLidClosed()))

        print("About to start: PourMaterial")
        get_input()
        c_rob_req.PourMaterial() 

        print("About to start: go2Hopper")
        get_input()
        c_rob_req.go2Hopper() 

        print("Attempting to close the hopper Lid")
        c_io.closeHopperLid()
        time.sleep(3)
        print("is Hopper lid open? Answer: " +  str(c_io.isHopperLidOpen()))
        print("is Hopper lid closed? Answer: " +  str(c_io.isHopperLidClosed()))

        print("About to start: go2CabinetLower")
        get_input()
        c_rob_req.go2CabinetLower()

        print("Attempting to open the cabinet Robo Door")
        c_io.openCabinetRoboDoor()
        time.sleep(4)
        print("is Cabinet Robo Door open? Answer: " +  str(c_io.isCabinetRoboDoorOpen()))
        print("is Cabinet robo door closed? Answer: " +  str(c_io.isCabinetRoboDoorClosed()))

        # Check for available places here
        # TODO

        print("About to start: go2Place(" + str(places_order[i]) + ")")
        get_input()
        c_rob_req.go2Place(places_order[i])

        print("Attempting to open the gripper")
        c_io.openGripper()
        time.sleep(2)
        print("is gripper open? Answer: " +  str(c_io.isGripperOpen()))
        print("is gripper closed? Answer: " +  str(c_io.isGripperClosed()))    

        print("About to start: go2CabinetLower")
        get_input()
        c_rob_req.go2CabinetLower()

        # Confirm correct placing here
        # TODO

        print("Attempting to close the cabinet Robo Door")
        c_io.closeCabinetRoboDoor()
        time.sleep(3)
        print("is Cabinet Robo Door open? Answer: " +  str(c_io.isCabinetRoboDoorOpen()))
        print("is Cabinet robo door closed? Answer: " +  str(c_io.isCabinetRoboDoorClosed()))

        print("About to start: go2Home")
        get_input()
        c_rob_req.go2Home()


def get_input():
    if (sys.version_info > (3, 0)):
        # input("press enter to continue...\n")
        print("press enter to continue...\n")
    else:
        # raw_input("press enter to continue...")
        print("press enter to continue...")

if __name__ == '__main__':
    main()