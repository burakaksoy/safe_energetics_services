#Simple example Robot Raconteur PlaceDetector client
#This program ...
from RobotRaconteur.Client import *

#Main program
def main():

    # url='rr+tcp://localhost:9003/?service=PlaceDetector'
    # url='rr+tcp://192.168.1.142:9003/?service=PlaceDetector'
    url='rr+tcp://192.168.55.11:9003/?service=PlaceDetector'

    #Start up Robot Raconteur and connect, standard by this point    
    c = RRN.ConnectService(url)

    available_locations = c.getAvailableLocations()
    print("Available locations:")
    for i in available_locations:
        print(i)

    print("Number of available locations:")
    print(len(available_locations))

if __name__ == '__main__':
    main()