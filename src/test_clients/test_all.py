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

    url_level='rr+tcp://192.168.55.140:9001/?service=LevelSensor' # 192.168.55.10
    url_lid='rr+tcp://192.168.55.142:9004/?service=LidChecker' # 192.168.55.11
    url_place='rr+tcp://192.168.55.142:9003/?service=PlaceDetector' # 192.168.55.11
    url_cup='rr+tcp://192.168.55.142:9002/?service=CupDetector' # 192.168.55.11

    RRN.RequestTimeout = 120 # seconds

    #Start up Robot Raconteur and connect, standard by this point    
    c_io = RRN.ConnectService(url_io)
    c_rob_req = RRN.ConnectService(url_rob_req)
    c_level = RRN.ConnectService(url_level)
    c_lid = RRN.ConnectService(url_lid)
    c_place = RRN.ConnectService(url_place)
    c_cup = RRN.ConnectService(url_cup)

    # -------------------------------------------------------

    print("About to start: LEVEL SENSOR SERVICE FUNCTIONS")
    level_low = c_level.isLevelLow()
    if level_low:
        print("Hopper level is LOW")
    else:
        print("Hopper level is GOOD")

    level_high = c_level.isLevelHigh()
    if level_high:
        print("Hopper level is HIGH")
    else:
        print("Hopper level is not HIGH, more material can be added")

    # TODO: wait for level to be low and and keep pouring until the level is high later
    # -------------------------------------------------------

    print("About to start: go2Home")
    get_input()
    c_rob_req.go2Home()

    print("is Cabinet Operator Door closed? Answer: " + str(c_io.isCabinetOpDoorClosed()))

    # TODO: act according to cabinet Operator door opening situation

    print("Attempting to close the cabinet Robo Door")
    c_io.closeCabinetRoboDoor()
    time.sleep(3)
    print("is Cabinet Robo Door open? Answer: " +  str(c_io.isCabinetRoboDoorOpen()))
    print("is Cabinet robo door closed? Answer: " +  str(c_io.isCabinetRoboDoorClosed()))

    # TODO: act according to cabinet Robo Door opening situation
    
    print("Attempting to close the hopper Lid")
    c_io.closeHopperLid()
    time.sleep(3)
    print("is Hopper lid open? Answer: " +  str(c_io.isHopperLidOpen()))
    print("is Hopper lid closed? Answer: " +  str(c_io.isHopperLidClosed()))

    # TODO: act according to hopper lid opening situation

    print("Attempting to open the gripper")
    c_io.openGripper()
    time.sleep(2)
    print("is gripper open? Answer: " +  str(c_io.isGripperOpen()))
    print("is gripper closed? Answer: " +  str(c_io.isGripperClosed()))

    # TODO: act according to gripper opening situation

    # TODO: Execute every function as a try-except method for error checking
    print("About to start: go2CabinetUpper")
    get_input()
    c_rob_req.go2CabinetUpper()

    print("Attempting to open the cabinet Robo Door")
    c_io.openCabinetRoboDoor()
    time.sleep(3)
    print("is Cabinet Robo Door open? Answer: " +  str(c_io.isCabinetRoboDoorOpen()))
    print("is Cabinet robo door closed? Answer: " +  str(c_io.isCabinetRoboDoorClosed()))

    # TODO: act according to gripper opening situation

    # Check for available cups to pickup here
    print("Getting the available cup locations..")
    cup_locations = c_cup.getCupLocations()
    print("Cup locations: " + str(cup_locations) )
    print("Number of cups: " + str(len(cup_locations)))
    
    while len(cup_locations) > 0:
        print("About to start: go2Cup(" + str(cup_locations[0]) + ")")
        get_input()
        c_rob_req.go2Cup(cup_locations[0])

        print("Attempting to close the gripper")
        c_io.closeGripper()
        time.sleep(2)
        print("is gripper open? Answer: " +  str(c_io.isGripperOpen()))
        print("is gripper closed? Answer: " +  str(c_io.isGripperClosed()))

        print("About to start: go2CabinetUpper")
        get_input()
        c_rob_req.go2CabinetUpper()

        # Check for graspin here
        print("Starting LID CHECKER SERVICE FUNCTIONS")
        lid_off = c_lid.isLidOff()
        if lid_off:
            print("Lid IS OFF from the cup")
        else:
            print("Lid IS NOT OFF from the cup")

        lid_on = c_lid.isLidOn()
        if lid_on:
            print("Lid IS ON the cup")
        else:
            print("Lid IS NOT ON the cup")

        grasped_correctly = c_lid.isGraspedCorrectly()
        if grasped_correctly:
            print("Cup is grasped correctly")
        else:
            print("Cup is NOT grasped correctly")

        # TODO: act according to cup grasping situation

        print("Attempting to close the cabinet Robo Door")
        get_input()
        c_io.closeCabinetRoboDoor()
        time.sleep(3)
        print("is Cabinet Robo Door open? Answer: " +  str(c_io.isCabinetRoboDoorOpen()))
        print("is Cabinet robo door closed? Answer: " +  str(c_io.isCabinetRoboDoorClosed()))

        print("About to start: go2LidRemover")
        get_input()
        c_rob_req.go2LidRemover()  

        print("About to start: RemoveCupLid")
        c_rob_req.RemoveCupLid()  

        # Check for cup lid removal here
        print("Starting LID CHECKER SERVICE FUNCTIONS")
        lid_off = c_lid.isLidOff()
        if lid_off:
            print("Lid IS OFF from the cup")
        else:
            print("Lid IS NOT OFF from the cup")

        lid_on = c_lid.isLidOn()
        if lid_on:
            print("Lid IS ON the cup")
        else:
            print("Lid IS NOT ON the cup")

        grasped_correctly = c_lid.isGraspedCorrectly()
        if grasped_correctly:
            print("Cup is grasped correctly")
        else:
            print("Cup is NOT grasped correctly")

        # TODO: act according to lid removal situation (eg try 3 times etc.)

        print("About to start: go2Hopper")
        get_input()
        c_rob_req.go2Hopper() 

        print("Attempting to open the hopper Lid")
        c_io.openHopperLid()
        time.sleep(3)
        print("is Hopper lid open? Answer: " +  str(c_io.isHopperLidOpen()))
        print("is Hopper lid closed? Answer: " +  str(c_io.isHopperLidClosed()))

        # TODO: act according to hopper lid opening situation

        print("About to start: PourMaterial")
        get_input()
        c_rob_req.PourMaterial() 

        print("About to start: go2Hopper")
        # get_input()
        c_rob_req.go2Hopper() 

        print("Attempting to close the hopper Lid")
        c_io.closeHopperLid()
        time.sleep(3)
        print("is Hopper lid open? Answer: " +  str(c_io.isHopperLidOpen()))
        print("is Hopper lid closed? Answer: " +  str(c_io.isHopperLidClosed()))

        # TODO: act according to hopper lid opening situation

        print("About to start: go2CabinetLower")
        # get_input()
        c_rob_req.go2CabinetLower()

        print("Attempting to open the cabinet Robo Door")
        c_io.openCabinetRoboDoor()
        time.sleep(4)
        print("is Cabinet Robo Door open? Answer: " +  str(c_io.isCabinetRoboDoorOpen()))
        print("is Cabinet robo door closed? Answer: " +  str(c_io.isCabinetRoboDoorClosed()))

        # TODO: act according to cabinet opening situation        

        # Check for available places here
        print("Getting the available place locations")
        available_locations = c_place.getAvailableLocations()
        print("Available locations: " + str(available_locations))
        print("Number of available locations: " + str(len(available_locations)))

        if not (len(available_locations) > 0):
            print("ERROR: No available location is left to put the cup back!")
            print("Exiting..")
            return
        else:
            placing_location = available_locations[0]
            print("About to start: go2Place(" + str(placing_location) + ")")
            get_input()
            c_rob_req.go2Place(placing_location)

            print("Attempting to open the gripper")
            c_io.openGripper()
            time.sleep(2)
            print("is gripper open? Answer: " +  str(c_io.isGripperOpen()))
            print("is gripper closed? Answer: " +  str(c_io.isGripperClosed()))    

            # TODO: act according to gripper opening situation

            print("About to start: go2CabinetLower")
            get_input()
            c_rob_req.go2CabinetLower()

            # Confirm correct placing here
            print("Getting the available place locations to confirm correct placing")
            available_locations_new = c_place.getAvailableLocations()
            print("Available locations new: " + str(available_locations_new))
            print("Number of new available locations: " + str(len(available_locations_new)))
            # difference between the old available locations and new available locations must be equal to the placing location

            # To get elements which are in available_locations but not in available_locations_new:
            difference_locations = list(set(available_locations) - set(available_locations_new))

            if not ((len(difference_locations) == 1) and (difference_locations[0] == placing_location)):
                print("ERROR: Cup could not be placed correctly!!!")
                print("Exiting...")
                return
            else:
                print("Cup placed correctly")

            print("Attempting to close the cabinet Robo Door")
            c_io.closeCabinetRoboDoor()
            time.sleep(3)
            print("is Cabinet Robo Door open? Answer: " +  str(c_io.isCabinetRoboDoorOpen()))
            print("is Cabinet robo door closed? Answer: " +  str(c_io.isCabinetRoboDoorClosed()))

            # TODO: act according to cabinet opening situation

            print("About to start: go2Home")
            get_input()
            c_rob_req.go2Home()

            # TODO: Check if more material is needed here
            # currently, assuming the material is always needed
            print("About to start: go2CabinetUpper")
            get_input()
            c_rob_req.go2CabinetUpper()

            print("Attempting to open the cabinet Robo Door")
            c_io.openCabinetRoboDoor()
            time.sleep(3)
            print("is Cabinet Robo Door open? Answer: " +  str(c_io.isCabinetRoboDoorOpen()))
            print("is Cabinet robo door closed? Answer: " +  str(c_io.isCabinetRoboDoorClosed()))

            # TODO: act according to gripper opening situation

            # Check for available cups to pickup here
            print("Getting the available cup locations..")
            cup_locations = c_cup.getCupLocations()
            print("Cup locations: " + str(cup_locations) )
            print("Number of cups: " + str(len(cup_locations)))
    
    print("No more available cups left in the cabinet")
    # TODO: act accordingly
    print("Exiting..")


def get_input():
    if (sys.version_info > (3, 0)):
        input("press enter to continue...\n")
        # print("press enter to continue...\n")
    else:
        raw_input("press enter to continue...")
        # print("press enter to continue...")

if __name__ == '__main__':
    main()