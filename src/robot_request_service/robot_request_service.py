import argparse
import sys
import pandas as pd
import os
import time

import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 

import numpy as np
from general_robotics_toolbox import *
from abb_motion_program_exec_client import *

import igraph as ig
import matplotlib.pyplot as plt

class RobotRequest_impl():
    def __init__(self,parameter_file,robot_url):
        self.file_name = parameter_file

        # # read csv files
        # self.file_name = os.path.expanduser(self.file_name)
        # self.df = pd.read_csv(self.file_name)
        # # self.inputs = self.df.values.tolist()
        # self.inputs = pd.Series(self.df.pin.values,index=self.df.name).to_dict()


        # print("Input pins: ")
        # print(str(self.inputs))

        # # Create helper objects
        # self.rapid = RAPID(base_url=robot_url)
        # self.gripper = Gripper(self.rapid,self.inputs,self.outputs)
        # self.cabinet = Cabinet(self.rapid,self.inputs,self.outputs)
        # self.hopper = Hopper(self.rapid,self.inputs,self.outputs)
        # self.gate = Gate(self.rapid,self.inputs,self.outputs)
        # self.emergency = Emergency(self.rapid,self.inputs,self.outputs)
        
        print("hello, creating a graph")
        # Construct a directed graph with n vertices

        vertices = [{'name': 'J_HOME'}, 
                    {'name': 'CL_1'},
                    {'name': 'CL_1B_D'},
                    {'name': 'CL_1B_U'},
                    {'name': 'CL_1F_D'},
                    {'name': 'CL_1F_U'},
                    {'name': 'CL_2'},
                    {'name': 'CL_2B_D'},
                    {'name': 'CL_2B_U'},
                    {'name': 'CL_2F_D'},
                    {'name': 'CL_2F_U'},
                    {'name': 'CL_3'},
                    {'name': 'CL_3B_D'},
                    {'name': 'CL_3B_U'},
                    {'name': 'CL_3F_D'},
                    {'name': 'CL_3F_U'},
                    {'name': 'CL_4'},
                    {'name': 'CL_4B_D'},
                    {'name': 'CL_4B_U'},
                    {'name': 'CL_4F_D'},
                    {'name': 'CL_4F_U'},
                    {'name': 'CL_5'},
                    {'name': 'CL_5B_D'},
                    {'name': 'CL_5B_U'},
                    {'name': 'CL_5F_D'},
                    {'name': 'CL_5F_U'},
                    {'name': 'CL_6'},
                    {'name': 'CL_6B_D'},
                    {'name': 'CL_6B_U'},
                    {'name': 'CL_6F_D'},
                    {'name': 'CL_6F_U'},
                    {'name': 'CU_0'},
                    {'name': 'CU_1'},
                    {'name': 'CU_1B_D'},
                    {'name': 'CU_1B_U'},
                    {'name': 'CU_1F_D'},
                    {'name': 'CU_1F_U'},
                    {'name': 'CU_2'},
                    {'name': 'CU_2B_D'},
                    {'name': 'CU_2B_U'},
                    {'name': 'CU_2F_D'},
                    {'name': 'CU_2F_U'},
                    {'name': 'CU_3'},
                    {'name': 'CU_3B_D'},
                    {'name': 'CU_3B_U'},
                    {'name': 'CU_3F_D'},
                    {'name': 'CU_3F_U'},
                    {'name': 'CU_4'},
                    {'name': 'CU_4B_D'},
                    {'name': 'CU_4B_U'},
                    {'name': 'CU_4F_D'},
                    {'name': 'CU_4F_U'},
                    {'name': 'CU_5'},
                    {'name': 'CU_5B_D'},
                    {'name': 'CU_5B_U'},
                    {'name': 'CU_5F_D'},
                    {'name': 'CU_5F_U'},
                    {'name': 'CU_6'},
                    {'name': 'CU_6B_D'},
                    {'name': 'CU_6B_U'},
                    {'name': 'CU_6F_D'},
                    {'name': 'CU_6F_U'},
                    {'name': 'H_0'},
                    {'name': 'H_1'},
                    {'name': 'H_2'},
                    {'name': 'H_3'},
                    {'name': 'UNC_0'},
                    {'name': 'UNC_1'},
                    {'name': 'UNC_2'},
                    {'name': 'UNC_3'}
                ] 
        edges = [   {'source': 'J_HOME',    'target': 'CU_0',   'motion': 'moveJ',      'speed': 'v500', 'wait': '0.5'}, 
                    {'source': 'CU_0',      'target': 'J_HOME', 'motion': 'moveAbsJ',   'speed': 'v500', 'wait': '0'}, 
                    {'source': 'CU_0',      'target': 'UNC_0',  'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'}, 
                    {'source': 'UNC_0',     'target': 'CU_0',   'motion': 'moveL',      'speed': 'v500', 'wait': '0'}, 
                    {'source': 'UNC_0',     'target': 'UNC_1',  'motion': 'moveL',      'speed': 'v100', 'wait': '0.5'}, 
                    {'source': 'UNC_1',     'target': 'UNC_2',  'motion': 'moveL',      'speed': 'v100', 'wait': '0.5'}, 
                    {'source': 'UNC_2',     'target': 'UNC_3',  'motion': 'moveL',      'speed': 'v100', 'wait': '0.5'}, 
                    {'source': 'UNC_3',     'target': 'UNC_0',  'motion': 'moveL',      'speed': 'v100', 'wait': '0.5'},

                    {'source': 'J_HOME',    'target': 'H_0',    'motion': 'moveJ',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'H_0',       'target': 'J_HOME', 'motion': 'moveAbsJ',   'speed': 'v500', 'wait': '0'},  
                    {'source': 'J_HOME',    'target': 'CL_0',   'motion': 'moveJ',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CL_0',      'target': 'J_HOME', 'motion': 'moveAbsJ',   'speed': 'v500', 'wait': '0'}, 

                    {'source': 'CU_0',      'target': 'CU_1',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CU_1',      'target': 'CU_0',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CU_0',      'target': 'CU_2',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CU_2',      'target': 'CU_0',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CU_0',      'target': 'CU_3',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CU_3',      'target': 'CU_0',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CU_0',      'target': 'CU_4',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CU_4',      'target': 'CU_0',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CU_0',      'target': 'CU_5',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CU_5',      'target': 'CU_0',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CU_0',      'target': 'CU_6',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CU_6',      'target': 'CU_0',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},

                    {'source': 'CL_0',      'target': 'CL_1',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CL_1',      'target': 'CL_0',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CL_0',      'target': 'CL_2',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CL_2',      'target': 'CL_0',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CL_0',      'target': 'CL_3',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CL_3',      'target': 'CL_0',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CL_0',      'target': 'CL_4',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CL_4',      'target': 'CL_0',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CL_0',      'target': 'CL_5',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CL_5',      'target': 'CL_0',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CL_0',      'target': 'CL_6',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},
                    {'source': 'CL_6',      'target': 'CL_0',   'motion': 'moveL',      'speed': 'v500', 'wait': '0.5'},

                    {'source': 'CU_1',      'target': 'CU_1F_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_1',      'target': 'CU_1B_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_1F_U',   'target': 'CU_1',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_1B_U',   'target': 'CU_1',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_2',      'target': 'CU_2F_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_2',      'target': 'CU_2B_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_2F_U',   'target': 'CU_2',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_2B_U',   'target': 'CU_2',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_3',      'target': 'CU_3F_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_3',      'target': 'CU_3B_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_3F_U',   'target': 'CU_3',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_3B_U',   'target': 'CU_3',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_4',      'target': 'CU_4F_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_4',      'target': 'CU_4B_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_4F_U',   'target': 'CU_4',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_4B_U',   'target': 'CU_4',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_5',      'target': 'CU_5F_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_5',      'target': 'CU_5B_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_5F_U',   'target': 'CU_5',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_5B_U',   'target': 'CU_5',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_6',      'target': 'CU_6F_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_6',      'target': 'CU_6B_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_6F_U',   'target': 'CU_6',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_6B_U',   'target': 'CU_6',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},

                    {'source': 'CU_1F_U',   'target': 'CU_1F_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_1F_D',   'target': 'CU_1F_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_1B_U',   'target': 'CU_1B_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_1B_D',   'target': 'CU_1B_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_2F_U',   'target': 'CU_2F_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_2F_D',   'target': 'CU_2F_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_2B_U',   'target': 'CU_2B_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_2B_D',   'target': 'CU_2B_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_3F_U',   'target': 'CU_3F_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_3F_D',   'target': 'CU_3F_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_3B_U',   'target': 'CU_3B_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_3B_D',   'target': 'CU_3B_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_4F_U',   'target': 'CU_4F_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_4F_D',   'target': 'CU_4F_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_4B_U',   'target': 'CU_4B_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_4B_D',   'target': 'CU_4B_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_5F_U',   'target': 'CU_5F_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_5F_D',   'target': 'CU_5F_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_5B_U',   'target': 'CU_5B_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_5B_D',   'target': 'CU_5B_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_6F_U',   'target': 'CU_6F_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_6F_D',   'target': 'CU_6F_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_6B_U',   'target': 'CU_6B_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CU_6B_D',   'target': 'CU_6B_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},


                    {'source': 'CL_1',      'target': 'CL_1F_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_1',      'target': 'CL_1B_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_1F_U',   'target': 'CL_1',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_1B_U',   'target': 'CL_1',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_2',      'target': 'CL_2F_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_2',      'target': 'CL_2B_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_2F_U',   'target': 'CL_2',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_2B_U',   'target': 'CL_2',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_3',      'target': 'CL_3F_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_3',      'target': 'CL_3B_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_3F_U',   'target': 'CL_3',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_3B_U',   'target': 'CL_3',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_4',      'target': 'CL_4F_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_4',      'target': 'CL_4B_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_4F_U',   'target': 'CL_4',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_4B_U',   'target': 'CL_4',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_5',      'target': 'CL_5F_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_5',      'target': 'CL_5B_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_5F_U',   'target': 'CL_5',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_5B_U',   'target': 'CL_5',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_6',      'target': 'CL_6F_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_6',      'target': 'CL_6B_U',    'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_6F_U',   'target': 'CL_6',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_6B_U',   'target': 'CL_6',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},

                    {'source': 'CL_1F_U',   'target': 'CL_1F_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_1F_D',   'target': 'CL_1F_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_1B_U',   'target': 'CL_1B_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_1B_D',   'target': 'CL_1B_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_2F_U',   'target': 'CL_2F_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_2F_D',   'target': 'CL_2F_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_2B_U',   'target': 'CL_2B_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_2B_D',   'target': 'CL_2B_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_3F_U',   'target': 'CL_3F_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_3F_D',   'target': 'CL_3F_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_3B_U',   'target': 'CL_3B_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_3B_D',   'target': 'CL_3B_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_4F_U',   'target': 'CL_4F_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_4F_D',   'target': 'CL_4F_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_4B_U',   'target': 'CL_4B_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_4B_D',   'target': 'CL_4B_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_5F_U',   'target': 'CL_5F_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_5F_D',   'target': 'CL_5F_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_5B_U',   'target': 'CL_5B_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_5B_D',   'target': 'CL_5B_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_6F_U',   'target': 'CL_6F_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_6F_D',   'target': 'CL_6F_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_6B_U',   'target': 'CL_6B_D',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                    {'source': 'CL_6B_D',   'target': 'CL_6B_U',       'motion': 'moveL',      'speed': 'v500', 'wait': '0'},
                
                    {'source': 'H_0',     'target': 'H_1',   'motion': 'moveL',      'speed': 'v500', 'wait': '0'}, 
                    {'source': 'H_1',     'target': 'H_0',   'motion': 'moveJ',      'speed': 'v500', 'wait': '0'}, 

                    {'source': 'H_1',     'target': 'H_2',   'motion': 'moveL',      'speed': 'v500', 'wait': '0'}, 
                    {'source': 'H_2',     'target': 'H_1',   'motion': 'moveL',      'speed': 'v500', 'wait': '0'}, 

                    {'source': 'H_2',     'target': 'H_3',   'motion': 'moveJ',      'speed': 'v500', 'wait': '1.5'}, 
                    {'source': 'H_3',     'target': 'H_2',   'motion': 'moveJ',      'speed': 'v500', 'wait': '0'} 
                ]
        g = ig.Graph.DictList(vertices, edges,directed=True)

        g.write_graphml("graph")
        
        # Set attributes for the graph, nodes, and edges
        g["title"] = "Waypoint Network"
        print(g)


        # Create the motion primitive code for shorthest path
        path_v = g.get_shortest_paths("UNC_1",to="J_HOME", output="vpath")[0][1:]
        # print(path_v)
        path_e = g.get_shortest_paths("UNC_1",to="J_HOME", output="epath")[0]
        # print(path_e)

        motion_code = []
        for v,e in zip(path_v,path_e):
            cmd = "mp." + g.es[e]["motion"] + "(" + g.vs[v]["name"] + "," + g.es[e]["speed"] + ",fine)"
            motion_code.append(cmd)
            if g.es[e]["wait"] != "0":
                cmd = "mp.WaitTime(" + g.es[e]["wait"] + ")"
                motion_code.append(cmd)
        
        for cmd in motion_code:
            print(cmd)


        

        


        # print("g.to_dict_list():")
        # print(g.to_dict_list())

        # Plot in matplotlib
        # Note that attributes can be set globally (e.g. vertex_size), or set individually using arrays (e.g. vertex_color)
        fig, ax = plt.subplots(figsize=(45,45))
        ig.plot(
            g,
            target=ax,
            layout="tree", # print nodes in a circular layout # kk, circle,  tree, fr
            vertex_size=0.85,
            vertex_shape="circle",
            vertex_color="white" ,
            vertex_frame_width=0.4,
            # vertex_frame_color="black",
            vertex_label=g.vs["name"],
            vertex_label_size=5.0,
            edge_curved=0,
            edge_width=1 # thickness of edge arrow
            # edge_arrow_size=0.1,
            # edge_arrow_width=0.1
            # edge_label= list(map('\n'.join, zip(g.es["motion"], g.es["speed"],g.es["wait"]))),
            # edge_label_size=7.0,
            # edge_align_label=True,
            # edge_color='#666'
            # edge_background='white'
        )
        plt.show()

        


def main():
    port_num = 9006
    robot_url = 'http://192.168.55.1:80' # ABB computer's URL
    parameter_file = "./robot_request_service.csv"
    
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.robot_request_service", port_num) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.robot_request_service")

        # create object
        RobotRequest_inst = RobotRequest_impl(parameter_file,robot_url)
        # register service with service name "RobotRequest", type "experimental.robot_request_service.RobotRequest", actual object: RobotRequest_inst
        RRN.RegisterService("RobotRequest","experimental.robot_request_service.RobotRequest",RobotRequest_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("robot_request_service started, press enter to quit...\n")
        else:
            raw_input("robot_request_service started, press enter to quit...")

if __name__ == '__main__':
    main()