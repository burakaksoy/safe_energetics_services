#Simple example Robot Raconteur RobotRequest client
#This program ...
from RobotRaconteur.Client import *

import time
import sys

def handler(err):
    if (err is not None):
        # If "err" is not None it means that an exception occurred.
        # "err" contains the exception object
        print ("An error occured! " + str(err))
        return
    print ("OK")

#Main program
def main():

    # url='rr+tcp://localhost:9006/?service=RobotRequest'
    url='rr+tcp://192.168.1.85:9006/?service=RobotRequest'
    # url='rr+tcp://192.168.55.11:9006/?service=RobotRequest'

    RRN.RequestTimeout = 60 # seconds

    #Start up Robot Raconteur and connect, standard by this point    
    c = RRN.ConnectService(url)
    # time.sleep(3)
    while True:
        inp = get_input()
        print('c.async_jog_to(\''+ str(inp) + '\',handler)')
        eval('c.async_jog_to(\''+ str(inp) + '\',handler)')



def get_input():
    if (sys.version_info > (3, 0)):
        inp = input("press enter which waypoint you want to jog to (e.g. J_HOME):\n")
    else:
        inp = raw_input("press enter which waypoint you want to jog to (e.g. J_HOME):")

    return inp

if __name__ == '__main__':
    main()