#Simple example Robot Raconteur PlaceDetector client
#This program ...
from RobotRaconteur.Client import *

import sys


# url='rr+tcp://localhost:9003/?service=PlaceDetector'
url='rr+tcp://192.168.1.142:9003/?service=PlaceDetector'
# url='rr+tcp://192.168.55.11:9003/?service=PlaceDetector'

#Main program
def main():
    argc = len(sys.argv)
    cmd = ""
    
    if argc == 2:
        cmd = sys.argv[1]
        
    if len(cmd) > 0:
        #Start up Robot Raconteur and connect, standard by this point    
        c = RRN.ConnectService(url)

        if cmd == "getAvailableLocations()" :
            available_locations = c.getAvailableLocations()
            print(available_locations)

if __name__ == '__main__':
    main()