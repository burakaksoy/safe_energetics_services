import argparse
import sys
import pandas as pd
import os
import time

import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 

from io_interface import RAPID
from helper_gripper import Gripper
from helper_cabinet import Cabinet
from helper_hopper import Hopper
from helper_gate import Gate
from helper_emergency import Emergency

class IOManager_impl():
    def __init__(self,inputs_file,outputs_file,IO_module_url):
        self.file_name_inputs = inputs_file
        self.file_name_outputs = outputs_file

        # read csv files
        self.file_name_inputs = os.path.expanduser(self.file_name_inputs)
        self.df_inputs = pd.read_csv(self.file_name_inputs)
        # self.inputs = self.df_inputs.values.tolist()
        self.inputs = pd.Series(self.df_inputs.pin.values,index=self.df_inputs.name).to_dict()

        self.file_name_outputs = os.path.expanduser(self.file_name_outputs)
        self.df_outputs = pd.read_csv(self.file_name_outputs)
        # self.outputs = self.df_outputs.values.tolist()
        self.outputs = pd.Series(self.df_outputs.pin.values,index=self.df_outputs.name).to_dict()


        print("Input pins: ")
        print(str(self.inputs))
        print("Output pins: ")
        print(str(self.outputs))

        # Create helper objects
        self.rapid = RAPID(base_url=IO_module_url)
        self.gripper = Gripper(self.rapid,self.inputs,self.outputs)
        self.cabinet = Cabinet(self.rapid,self.inputs,self.outputs)
        self.hopper = Hopper(self.rapid,self.inputs,self.outputs)
        self.gate = Gate(self.rapid,self.inputs,self.outputs)
        self.emergency = Emergency(self.rapid,self.inputs,self.outputs)

    def openGripper(self):
        self.gripper.open()

    def isGripperOpen(self):
        return self.gripper.isOpen()

    def closeGripper(self):
        self.gripper.close()

    def isGripperClosed(self):
        return self.gripper.isClosed()


    def openCabinetRoboDoor(self):
        self.cabinet.openRoboDoor()

    def isCabinetRoboDoorOpen(self):
        return self.cabinet.isRoboDoorOpen()

    def closeCabinetRoboDoor(self):
        self.cabinet.closeRoboDoor()

    def isCabinetRoboDoorClosed(self):
        return self.cabinet.isRoboDoorClosed()

    def isCabinetOpDoorClosed(self):
        return self.cabinet.isOpDoorClosed()


    def openHopperLid(self):
        self.hopper.openLid()

    def isHopperLidOpen(self):
        return self.hopper.isLidOpen()

    def closeHopperLid(self):
        self.hopper.closeLid()

    def isHopperLidClosed(self):
        return self.hopper.isLidClosed()

    def isHopperPressureGood(self):
        return self.hopper.isPressureGood()


    def lockGate(self):
        self.gate.lock()

    def isGateLocked(self):
        return self.gate.isLocked()

    def unlockGate(self):
        self.gate.unlock()

    def isGateClosed(self):
        return self.gate.isClosed()


    def isEmergency(self):
        return self.emergency.isEmergency()

        
        


def main():
    port_num = 9005
    IO_module_url = 'http://192.168.55.1:80' # ABB computer's URL
    inputs_file = "./io_manager_service_inputs.csv"
    outputs_file = "./io_manager_service_outputs.csv"
    
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.io_manager_service", port_num) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.io_manager_service")

        # create object
        IOManager_inst = IOManager_impl(inputs_file,outputs_file,IO_module_url)
        # register service with service name "IOManager", type "experimental.io_manager_service.IOManager", actual object: IOManager_inst
        RRN.RegisterService("IOManager","experimental.io_manager_service.IOManager",IOManager_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("io_manager_service started, press enter to quit...\n")
        else:
            raw_input("io_manager_service started, press enter to quit...")

if __name__ == '__main__':
    main()