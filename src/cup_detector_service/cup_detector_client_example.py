#Simple example Robot Raconteur CupDetector client
#This program ...
from RobotRaconteur.Client import *

#Main program
def main():

    # url='rr+tcp://localhost:9002/?service=CupDetector'
    # url='rr+tcp://192.168.1.142:9002/?service=CupDetector'
    url='rr+tcp://192.168.55.11:9002/?service=CupDetector'

    #Start up Robot Raconteur and connect, standard by this point    
    c = RRN.ConnectService(url)

    cup_locations = c.getCupLocations()
    print("Cup locations:")
    for i in cup_locations:
        print(i)

    print("Number of cups:")
    print(len(cup_locations))

if __name__ == '__main__':
    main()