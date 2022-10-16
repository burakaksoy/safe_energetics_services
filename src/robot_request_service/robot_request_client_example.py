#Simple example Robot Raconteur RobotRequest client
#This program ...
from RobotRaconteur.Client import *

import time

#Main program
def main():

    url='rr+tcp://localhost:9006/?service=RobotRequest'
    # url='rr+tcp://192.168.1.142:9006/?service=RobotRequest'

    #Start up Robot Raconteur and connect, standard by this point    
    c = RRN.ConnectService(url)

    print("Attempting to ...")
    # c.openGripper() # TODO
    time.sleep(1)


if __name__ == '__main__':
    main()