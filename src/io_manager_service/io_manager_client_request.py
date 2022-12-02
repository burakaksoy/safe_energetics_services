#Simple example Robot Raconteur CupDetector client
#This program ...
from RobotRaconteur.Client import *

import time
import sys

def main():
    argc = len(sys.argv)
    cmd = ""
    
    if argc == 2 :
        cmd = sys.argv[1]
    
    if len(cmd) > 0 :
        url='rr+tcp://localhost:9005/?service=IOManager'
        # url='rr+tcp://192.168.1.142:9005/?service=IOManager'
        # url='rr+tcp://192.168.55.11:9005/?service=IOManager'

        #Start up Robot Raconteur and connect, standard by this point    
        c = RRN.ConnectService(url)

        try: 
            result = eval('c.' + str(cmd))
            if result is None:
                result = "OK"
            print(result)
        except:
            print("FAILED")
            pass

if __name__ == '__main__':
    main()