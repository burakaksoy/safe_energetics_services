#Simple example Robot Raconteur LevelSensor client
#This program ...
from RobotRaconteur.Client import *

#Main program
def main():

    url='rr+tcp://localhost:9001/?service=LevelSensor'
    #Start up Robot Raconteur and connect, standard by this point    
    c = RRN.ConnectService(url)

    level_low = c.isLevelLow()
    if level_low:
        print("Hopper level is LOW")
    else:
        print("Hopper level is GOOD")


if __name__ == '__main__':
    main()