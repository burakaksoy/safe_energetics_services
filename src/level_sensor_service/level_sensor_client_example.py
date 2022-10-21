#Simple example Robot Raconteur LevelSensor client
#This program ...
from RobotRaconteur.Client import *

#Main program
def main():

    # url='rr+tcp://localhost:9001/?service=LevelSensor'
    # url='rr+tcp://192.168.1.140:9001/?service=LevelSensor'
    url='rr+tcp://192.168.55.10:9001/?service=LevelSensor'

    #Start up Robot Raconteur and connect, standard by this point    
    c = RRN.ConnectService(url)

    level_low = c.isLevelLow()
    if level_low:
        print("Hopper level is LOW")
    else:
        print("Hopper level is GOOD")

    level_high = c.isLevelHigh()
    if level_high:
        print("Hopper level is HIGH")
    else:
        print("Hopper level is not HIGH, more material can be added")


if __name__ == '__main__':
    main()