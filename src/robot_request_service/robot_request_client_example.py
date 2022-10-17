#Simple example Robot Raconteur RobotRequest client
#This program ...
from RobotRaconteur.Client import *

import time
import sys

#Main program
def main():

    url='rr+tcp://localhost:9006/?service=RobotRequest'
    # url='rr+tcp://192.168.1.142:9006/?service=RobotRequest'

    #Start up Robot Raconteur and connect, standard by this point    
    c = RRN.ConnectService(url)

    print("About to start: go2Home")
    get_input()
    c.go2Home()

    for i in range(1):
        print("About to start: go2CabinetUpper")
        get_input()
        c.go2CabinetUpper()

        print("About to start: go2Cup(" + str(i) + ")")
        get_input()
        c.go2Cup(i)

    print("About to start: go2CabinetUpper")
    get_input()
    c.go2CabinetUpper()    

    print("About to start: go2LidRemover")
    get_input()
    c.go2LidRemover()  

    print("About to start: RemoveCupLid")
    get_input()
    c.RemoveCupLid()  

    print("About to start: go2Hopper")
    get_input()
    c.go2Hopper() 

    print("About to start: PourMaterial")
    get_input()
    c.PourMaterial() 

    for i in range(1):
        print("About to start: go2CabinetLower")
        get_input()
        c.go2CabinetLower()

        print("About to start: go2Place(" + str(i) + ")")
        get_input()
        c.go2Place(i)

    print("About to start: go2CabinetLower")
    get_input()
    c.go2CabinetLower() 

    print("About to start: go2Home")
    get_input()
    c.go2Home()


def get_input():
    if (sys.version_info > (3, 0)):
        # input("press enter to continue...\n")
        print("press enter to continue...\n")
    else:
        # raw_input("press enter to continue...")
        print("press enter to continue...")

if __name__ == '__main__':
    main()