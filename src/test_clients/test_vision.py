#Simple example Robot Raconteur RobotRequest client
#This program ...
from RobotRaconteur.Client import *

import time
import sys

#Main program
def main():
    url_level='rr+tcp://192.168.1.140:9001/?service=LevelSensor' # 192.168.55.10
    url_lid='rr+tcp://192.168.1.142:9004/?service=LidChecker' # 192.168.55.11
    url_place='rr+tcp://192.168.1.142:9003/?service=PlaceDetector' # 192.168.55.11
    url_cup='rr+tcp://192.168.1.142:9002/?service=CupDetector' # 192.168.55.11

    RRN.RequestTimeout = 120 # seconds

    #Start up Robot Raconteur and connect, standard by this point    
    c_level = RRN.ConnectService(url_level)
    c_lid = RRN.ConnectService(url_lid)
    c_place = RRN.ConnectService(url_place)
    c_cup = RRN.ConnectService(url_cup)

    # -------------------------------------------------------

    print("About to start: CUP DETECTOR SERVICE FUNCTIONS")
    get_input()
    cup_locations = c_cup.getCupLocations()
    print("Cup locations:")
    for i in cup_locations:
        print(i)
    print("Number of cups:")
    print(len(cup_locations))

    # -------------------------------------------------------

    print("About to start: LID CHECKER SERVICE FUNCTIONS")
    get_input()
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

    # -------------------------------------------------------

    print("About to start: LEVEL SENSOR SERVICE FUNCTIONS")
    get_input()
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

    # -------------------------------------------------------

    print("About to start: PLACE DETECTOR SERVICE FUNCTIONS")
    get_input()
    available_locations = c_place.getAvailableLocations()
    print("Available locations:")
    for i in available_locations:
        print(i)

    print("Number of available locations:")
    print(len(available_locations))





def get_input():
    if (sys.version_info > (3, 0)):
        # input("press enter to continue...\n")
        print("press enter to continue...\n")
    else:
        # raw_input("press enter to continue...")
        print("press enter to continue...")

if __name__ == '__main__':
    main()