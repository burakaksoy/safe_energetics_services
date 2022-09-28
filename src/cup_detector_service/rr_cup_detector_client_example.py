#Simple example Robot Raconteur CupDetector client
#This program ...
from RobotRaconteur.Client import *

#Main program
def main():

    url='rr+tcp://localhost:9001/?service=CupDetector'
    #Start up Robot Raconteur and connect, standard by this point    
    c = RRN.ConnectService(url)

    places_of_cups = c.places_of_cups()
    for i in places_of_cups:
        print(i)

    num_of_cups = c.num_of_cups()
    print(num_of_cups)

if __name__ == '__main__':
    main()