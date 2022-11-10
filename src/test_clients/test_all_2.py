#Simple example Robot Raconteur RobotRequest client
#This program ...
from RobotRaconteur.Client import *

import time
import sys

# !!!!!!!!!!!!!!!!!!!!VERY IMPORTANT WARNING!!!!!!!!!!!!!!!!!!!
# BE READY TO STOP THE ROBOT WHEN RUNNING THIS SCRIPT!!!!!

#Main program
def main():
    url_io='rr+tcp://localhost:9005/?service=IOManager'
    url_rob_req='rr+tcp://localhost:9006/?service=RobotRequest'

    url_level='rr+tcp://192.168.1.140:9001/?service=LevelSensor' # 192.168.55.10
    url_lid='rr+tcp://192.168.1.142:9004/?service=LidChecker' # 192.168.55.11
    url_place='rr+tcp://192.168.1.142:9003/?service=PlaceDetector' # 192.168.55.11
    url_cup='rr+tcp://192.168.1.142:9002/?service=CupDetector' # 192.168.55.11

    RRN.RequestTimeout = 120 # seconds

    #Start up Robot Raconteur and connect, standard by this point    
    c_io = RRN.ConnectService(url_io)
    c_rob_req = RRN.ConnectService(url_rob_req)
    c_level = RRN.ConnectService(url_level)
    c_lid = RRN.ConnectService(url_lid)
    c_place = RRN.ConnectService(url_place)
    c_cup = RRN.ConnectService(url_cup)

    no_pouring = False # set True for testing cabinet cup locations quickly, w/out pouring motion
    no_hopper = True # set True for disabling hopper cover actuation

    # Assume here, the initializations are done on the GUI,
    # Operator has done the manual product load,
    # -------------------------------------------------------

    # Some initial checks and commands for safety
    print("is Cabinet Operator Door closed? Answer: " + str(c_io.isCabinetOpDoorClosed()))
    # TODO: act according to cabinet Operator door opening situation

    print("is gate Closed? Answer: " +  str(c_io.isGateClosed()))
    # TODO: Let the operator know that they should close the door

    print("Attempting to lock the gate")
    c_io.lockGate()
    time.sleep(1)
    print("is gate locked? Answer: " +  str(c_io.isGateLocked()))
    # TODO: Act according to the gate lock sitatuion

    # If there is a unlock gate request do it as below:
    # print("Attempting to unlock the gate")
    # c_io.unlockGate()
    # time.sleep(1)
    # print("is gate Locked? Answer: " +  str(c_io.isGateLocked()))

    # emergency stop IO checking is done like this:
    print("is there Emergency? Answer: " +  str(c_io.isEmergency()))
    # TODO: act according to emergency

    # Checking for hopper pressure as well:
    print("is Hopper Pressure good? Answer: " +  str(c_io.isHopperPressureGood()))

    print("Attempting to close the cabinet Robo Door")
    c_io.closeCabinetRoboDoor()
    time.sleep(3)
    print("is Cabinet Robo Door open? Answer: " +  str(c_io.isCabinetRoboDoorOpen()))
    print("is Cabinet robo door closed? Answer: " +  str(c_io.isCabinetRoboDoorClosed()))
    # TODO: act according to cabinet Robo Door opening situation
    
    if not no_hopper:
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

    # Assume everything went fine until here..

    # -------------------------------------------------------
    # wait for level to be low and and keep pouring until the level is high
    need_material = False # Initially assume no material is needed in the hopper
    while True: 
        print("Checking is hopper level Low?")
        level_low = c_level.isLevelLow()
        if level_low:
            print("Hopper level is LOW")
        else:
            print("Hopper level is GOOD")

        print("Checking is hopper level High?")
        level_high = c_level.isLevelHigh()
        if level_high:
            print("Hopper level is HIGH")
        else:
            print("Hopper level is not HIGH, more material can be added")

        if level_low:
            need_material = True
        elif level_high:
            need_material = False

        if need_material:
            print("Material filling is needed in the hopper")

            print("About to start: go2Home")
            # get_input()
            c_rob_req.go2Home()

            # TODO: Execute every function as a try-except method for error checking
            print("About to start: go2CabinetUpper")
            get_input()
            c_rob_req.go2CabinetUpper()

            print("Attempting to open the cabinet Robo Door")
            c_io.openCabinetRoboDoor()
            time.sleep(3)
            print("is Cabinet Robo Door open? Answer: " +  str(c_io.isCabinetRoboDoorOpen()))
            print("is Cabinet robo door closed? Answer: " +  str(c_io.isCabinetRoboDoorClosed()))
            # TODO: act according to cabinet robo door opening situation

            # Check for available cups to pickup here
            print("Getting the available cup locations..")
            cup_locations = c_cup.getCupLocations()
            print("Cup locations: " + str(cup_locations) )
            print("Number of cups: " + str(len(cup_locations)))
            
            if len(cup_locations) <= 0:
                print("No more available cups left in the cabinet")
                # TODO: act accordingly!!!!
                print("Exiting..")
            else: # Great, we have cups to pour!
                if len(cup_locations) < 2:
                    print("1 cup left in the cabinet! ")
                    # TODO: act accordingly, require op. acknowledge

                print("About to start: go2Cup(" + str(cup_locations[0]) + ")")
                # get_input()
                c_rob_req.go2Cup(cup_locations[0])

                print("Attempting to close the gripper")
                c_io.closeGripper()
                time.sleep(2)
                print("is gripper open? Answer: " +  str(c_io.isGripperOpen()))
                print("is gripper closed? Answer: " +  str(c_io.isGripperClosed()))
                # TODO: act according to gripper opening situation

                print("About to start: go2CabinetUpper")
                # get_input()
                c_rob_req.go2CabinetUpper()

                print("Attempting to close the cabinet Robo Door")
                # get_input()
                c_io.closeCabinetRoboDoor()
                time.sleep(3)
                print("is Cabinet Robo Door open? Answer: " +  str(c_io.isCabinetRoboDoorOpen()))
                print("is Cabinet robo door closed? Answer: " +  str(c_io.isCabinetRoboDoorClosed()))
                # TODO: act according to cabinet Robo Door opening situation

                # Check for graspin here
                print("Starting LID CHECKER SERVICE FUNCTIONS")
                lid_off = c_lid.isLidOff()
                lid_on = c_lid.isLidOn()
                grasped_correctly = c_lid.isGraspedCorrectly()

                # TODO: act according to cup grasping situation
                if not (grasped_correctly and lid_on and (not lid_off)):
                    if (not lid_on) and lid_off:
                        print("ERROR: Lid is NOT on the grasped cup!")
                    else:
                        print("ERROR: Cup is NOT grasped correctly!")
                    print("Exiting..")
                    break

                # else
                print("Cup is grasped correctly!")
                # print("we can go ahead and try removing the lid")

                # Lid removal attempts
                is_success_lid_removal = False
                for trial in range(3):
                    if not is_success_lid_removal:
                        print("About to start: go2LidRemover")
                        # get_input()
                        c_rob_req.go2LidRemover()  
                        print("About to start: RemoveCupLid")
                        c_rob_req.RemoveCupLid()  

                        # Check for cup lid removal here
                        print("Starting LID CHECKER SERVICE FUNCTIONS")
                        lid_off = c_lid.isLidOff()
                        lid_on = c_lid.isLidOn()
                        grasped_correctly = c_lid.isGraspedCorrectly()

                        if (grasped_correctly and lid_off and (not lid_on)):
                            is_success_lid_removal = True
                        else:
                            print("Cup is probably dropped while trying removing the lid")
                
                # TODO: act according to lid removal situation after trials
                if not is_success_lid_removal:
                    print("ERROR: Lid removal failed!")
                    print("Exiting..")
                    break

                # else:
                print("Lid removal is successful!")

                if not no_pouring: # test purposes only
                    print("About to start: go2Hopper")
                    # get_input()
                    c_rob_req.go2Hopper() 

                    print("Attempting to open the hopper Lid")
                    c_io.openHopperLid()
                    time.sleep(3)
                    print("is Hopper lid open? Answer: " +  str(c_io.isHopperLidOpen()))
                    print("is Hopper lid closed? Answer: " +  str(c_io.isHopperLidClosed()))
                    # TODO: act according to hopper lid opening situation

                    print("About to start: PourMaterial")
                    # get_input()
                    c_rob_req.PourMaterial() 
                    time.sleep(1)
                    print("About to start: go2Hopper")
                    # get_input()
                    c_rob_req.go2Hopper() 

                    if not no_hopper:
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
                time.sleep(3)
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
                    # get_input()
                    c_rob_req.go2Place(placing_location)

                    print("Attempting to open the gripper")
                    c_io.openGripper()
                    time.sleep(2)
                    print("is gripper open? Answer: " +  str(c_io.isGripperOpen()))
                    print("is gripper closed? Answer: " +  str(c_io.isGripperClosed()))    

                    # TODO: act according to gripper opening situation

                    print("About to start: go2CabinetLower")
                    # get_input()
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
                    # get_input()
                    c_rob_req.go2Home()

        else:
            print("No more material filling is needed in the hopper")


    # -------------------------------------------------------


def get_input():
    if (sys.version_info > (3, 0)):
        input("press enter to continue...\n")
        # print("press enter to continue...\n")
    else:
        raw_input("press enter to continue...")
        # print("press enter to continue...")

if __name__ == '__main__':
    main()