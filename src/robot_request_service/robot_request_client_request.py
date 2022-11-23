#Simple example Robot Raconteur RobotRequest client
#This program ...

## Commands Supported:
# go2Home()
# go2CabinetUpper()
# go2CabinetLower()
# go2Cup(i) - i is range of 1 to 12
# go2Place(i) - i is range of 1 to 12
# go2LidRemover()  
# RemoveCupLid()  
# go2Hopper() 
# PourMaterial()
    
from RobotRaconteur.Client import *

import time
import sys

#Main program
def main():
    RRN.RequestTimeout = 120 # seconds
    url='rr+tcp://localhost:9006/?service=RobotRequest'
    # url='rr+tcp://192.168.1.142:9006/?service=RobotRequest'
    # url='rr+tcp://192.168.55.11:9006/?service=RobotRequest'

    #Start up Robot Raconteur and connect, standard by this point    
    c = RRN.ConnectService(url)

    argc = len(sys.argv)
    cmd = ""
    
    if argc == 2 :
        cmd = sys.argv[1]
        
    if ( len(cmd) > 0 ):
        try: 
            result = eval('c.' + str(cmd))
            print("OK")
        except:
            print("FAILED")
            pass

if __name__ == '__main__':
    main()