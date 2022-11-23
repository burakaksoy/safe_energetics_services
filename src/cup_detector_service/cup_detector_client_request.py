#Simple example Robot Raconteur CupDetector client
#This program ...
from RobotRaconteur.Client import *

import sys

#Main program
def main():
    argc = len(sys.argv)
    cmd = ""
    
    if argc == 2 :
        cmd = sys.argv[1] + "()"
        
    if argc == 3 :
        cmd = sys.argv[1] + "(" + sys.argv[2] + ")"
    
    if len(cmd) > 0 :
        
        # url='rr+tcp://localhost:9002/?service=CupDetector'
        url='rr+tcp://192.168.1.142:9002/?service=CupDetector'
        # url='rr+tcp://192.168.55.11:9002/?service=CupDetector'

        #Start up Robot Raconteur and connect, standard by this point    
        c = RRN.ConnectService(url)

        if cmd == "getCupLocations()":
            try:
                cup_locations = c.getCupLocations()
                if cup_locations is not None:
                    print(cup_locations )
            except:
                print("Failed to Evaluate the requested command")
                pass

if __name__ == '__main__':
    main()