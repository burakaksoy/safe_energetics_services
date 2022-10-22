#Simple example Robot Raconteur RobotRequest client
#This program ...
from RobotRaconteur.Client import *

import time
import sys

#Main program
def main():

    url='rr+tcp://localhost:9006/?service=RobotRequest'
    # url='rr+tcp://192.168.1.142:9006/?service=RobotRequest'
    # url='rr+tcp://192.168.55.11:9006/?service=RobotRequest'

    #Start up Robot Raconteur and connect, standard by this point    
    c = RRN.ConnectService(url)

    while True:
        inp = get_input()
        print('c.jog_to(\''+ str(inp) + '\')')
        eval('c.jog_to(\''+ str(inp) + '\')')



def get_input():
    if (sys.version_info > (3, 0)):
        inp = input("press enter which waypoint you want to jog to (e.g. J_HOME):\n")
    else:
        inp = raw_input("press enter which waypoint you want to jog to (e.g. J_HOME):")

    return inp

if __name__ == '__main__':
    main()