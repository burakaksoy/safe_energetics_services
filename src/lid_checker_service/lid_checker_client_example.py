#Simple example Robot Raconteur LidChecker client
#This program ...
from RobotRaconteur.Client import *

#Main program
def main():

    # url='rr+tcp://localhost:9004/?service=LidChecker'
    url='rr+tcp://192.168.1.142:9004/?service=LidChecker'
    # url='rr+tcp://192.168.55.11:9004/?service=LidChecker'

    #Start up Robot Raconteur and connect, standard by this point    
    c = RRN.ConnectService(url)

    lid_off = c.isLidOff()
    if lid_off:
        print("Lid IS OFF from the cup")
    else:
        print("Lid IS NOT OFF from the cup")

    lid_on = c.isLidOn()
    if lid_on:
        print("Lid IS ON the cup")
    else:
        print("Lid IS NOT ON the cup")

    grasped_correctly = c.isGraspedCorrectly()
    if grasped_correctly:
        print("Cup is grasped correctly")
    else:
        print("Cup is NOT grasped correctly")

if __name__ == '__main__':
    main()