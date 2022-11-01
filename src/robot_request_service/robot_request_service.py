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
from robots_def import *

import igraph as ig
import matplotlib.pyplot as plt

class RobotRequest_impl():
    def __init__(self,parameter_file,robot_url):
        self.file_name = parameter_file
        self.robot_url = robot_url

        # this program is for single robot only, but a param is needed when ABB is using multimove controller
        self.is_multi_move = True 
        self.q2 = None # for joint angles of the second robot. 

        # Create motion program client
        self.mot_prog_client = MotionProgramExecClient(base_url=self.robot_url)
        # self.mot_prog_client = MotionProgramExecClient() # for simulation

        # Create robot tool data
        self.tool0 = tooldata(True,pose([5.5,0,270.7],[1,0,0,0]),loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0))   
        # Create robotics toolbox robot
        self.robot_rox = abb6640(R_tool=q2R([0.7071068,0,0.7071068,0]), p_tool=[270.7,0,-5.5]) # p_tool is defined wrt robot base

        # Speed parameters
        self.v_gnrl = "v300"
        self.v_pour = "v300"
        self.v_uncp = "v30" # v5, v10, v20, v30, v40, v50, v60, v80, v100, v200, v300, v400, v500, v600, v800, v1000, v1500, v2000, v2500
        
        print("Creating Waypoint Network graph..")
        # Construct a directed graph with n vertices
        self.vertices = [
                    {'name': 'J_HOME',  'pos': [1933.20,0.0,2049.5],        'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (0,0,0,0)       }, 
                    {'name': 'CL_0',    'pos': [1911.1,-500.0,1195.27],     'quat': [0.661949,0.0,0.7495489,0.0],  'conf': (-1,-1,0,1)     }, 
                    {'name': 'CL_1',    'pos': [2061.1,-238.062,1115.27],   'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)        },
                    {'name': 'CL_1B_D', 'pos': [2336.513,-238.062,1077.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_1B_U', 'pos': [2336.513,-238.062,1115.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_1F_D', 'pos': [2263.488,-238.062,1077.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_1F_U', 'pos': [2263.488,-238.062,1115.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_2',    'pos': [2061.1,-342.837,1115.27],   'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)        },
                    {'name': 'CL_2B_D', 'pos': [2336.513,-342.837,1077.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_2B_U', 'pos': [2336.513,-342.837,1115.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_2F_D', 'pos': [2263.488,-342.837,1077.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_2F_U', 'pos': [2263.488,-342.837,1115.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_3',    'pos': [2061.1,-447.612,1115.27],   'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)        },
                    {'name': 'CL_3B_D', 'pos': [2336.513,-447.612,1077.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_3B_U', 'pos': [2336.513,-447.612,1115.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_3F_D', 'pos': [2263.488,-447.612,1077.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_3F_U', 'pos': [2263.488,-447.612,1115.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_4',    'pos': [2061.1,-552.387,1115.27],   'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)        },
                    {'name': 'CL_4B_D', 'pos': [2336.513,-552.387,1077.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_4B_U', 'pos': [2336.513,-552.387,1115.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_4F_D', 'pos': [2263.488,-552.387,1077.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_4F_U', 'pos': [2263.488,-552.387,1115.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_5',    'pos': [2061.1,-657.162,1115.27],   'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)        },
                    {'name': 'CL_5B_D', 'pos': [2336.513,-657.162,1077.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_5B_U', 'pos': [2336.513,-657.162,1115.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_5F_D', 'pos': [2263.488,-657.162,1077.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_5F_U', 'pos': [2263.488,-657.162,1115.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_6',    'pos': [2061.1,-761.937,1115.27],   'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)        },
                    {'name': 'CL_6B_D', 'pos': [2336.513,-761.937,1077.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_6B_U', 'pos': [2336.513,-761.937,1115.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_6F_D', 'pos': [2263.488,-761.937,1077.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CL_6F_U', 'pos': [2263.488,-761.937,1115.27], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_0',    'pos': [1911.1,-500.0,1500.07],     'quat': [0.661949,0.0,0.7495489,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_1',    'pos': [2061.1,-238.062,1420.07],   'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)        },
                    {'name': 'CU_1B_D', 'pos': [2336.513,-238.062,1382.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_1B_U', 'pos': [2336.513,-238.062,1420.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_1F_D', 'pos': [2263.488,-238.062,1382.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_1F_U', 'pos': [2263.488,-238.062,1420.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_2',    'pos': [2061.1,-342.837,1420.07],   'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)        },
                    {'name': 'CU_2B_D', 'pos': [2336.513,-342.837,1382.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_2B_U', 'pos': [2336.513,-342.837,1420.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_2F_D', 'pos': [2263.488,-342.837,1382.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_2F_U', 'pos': [2263.488,-342.837,1420.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_3',    'pos': [2061.1,-447.612,1420.07],   'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)        },
                    {'name': 'CU_3B_D', 'pos': [2336.513,-447.612,1382.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_3B_U', 'pos': [2336.513,-447.612,1420.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_3F_D', 'pos': [2263.488,-447.612,1382.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_3F_U', 'pos': [2263.488,-447.612,1420.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_4',    'pos': [2061.1,-552.387,1420.07],   'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)        },
                    {'name': 'CU_4B_D', 'pos': [2336.513,-552.387,1382.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_4B_U', 'pos': [2336.513,-552.387,1420.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_4F_D', 'pos': [2263.488,-552.387,1382.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_4F_U', 'pos': [2263.488,-552.387,1420.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_5',    'pos': [2061.1,-657.162,1420.07],   'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)        },
                    {'name': 'CU_5B_D', 'pos': [2336.513,-657.162,1382.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_5B_U', 'pos': [2336.513,-657.162,1420.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_5F_D', 'pos': [2263.488,-657.162,1382.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_5F_U', 'pos': [2263.488,-657.162,1420.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_6',    'pos': [2061.1,-761.937,1420.07],   'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)        },
                    {'name': 'CU_6B_D', 'pos': [2336.513,-761.937,1382.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_6B_U', 'pos': [2336.513,-761.937,1420.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_6F_D', 'pos': [2263.488,-761.937,1382.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'CU_6F_U', 'pos': [2263.488,-761.937,1420.07], 'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'H_0',     'pos': [1334.925,-1000.0,2006.6],   'quat': [0.5,0.5,0.5,-0.5],                 'conf': (-1,-2,1,0)        },
                    {'name': 'H_1',     'pos': [1334.925,-1400.0,2006.6],   'quat': [0.5,0.5,0.5,-0.5],                 'conf': (-1,-2,1,0)        },
                    # {'name': 'H_2',     'pos': [1434.925,-1400.0,2106.6],   'quat': [0.5,0.5,0.5,-0.5],                 'conf': (-1,-1,0,0)        },
                    {'name': 'H_2',     'pos': [1385.30,-1400.0,1992.99],   'quat': [0.3248998,0.32490,0.62804,-0.62805],                 'conf': (-1,-1,0,0)        },
                    # {'name': 'H_3',     'pos': [1462.925,-1400.0,2106.6],   'quat': [0.5,0.5,-0.5,0.5],                 'conf': (-1,-1,-2,0)        },
                    {'name': 'H_3',     'pos': [1383.30,-1400.0,2047.48],   'quat': [0.0,0.0,0.7071068,-0.7071068],                 'conf': (-1,-1,-2,0)        },
                    {'name': 'H_4',     'pos': [1464.69,-1400.0,2086.1],   'quat': [0.5,0.5,-0.5,0.5],                 'conf': (-1,-1,-2,0)        },
                    {'name': 'UNC_0',   'pos': [2139.22,-395.974,572.17],     'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'UNC_1',   'pos': [2139.22,-288.59,572.17],     'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'UNC_2',   'pos': [2136.00,-288.59,572.17],     'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)      },
                    {'name': 'UNC_3',   'pos': [2136.00,-288.59,556.17],     'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)       },
                    {'name': 'UNC_4',   'pos': [1946.16,-288.59,556.17],     'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)       },
                    {'name': 'UNC_5',   'pos': [1946.16,-395.974,572.17],     'quat': [0.707106781,0.0,0.707106781,0.0],  'conf': (-1,-1,0,1)       }
                ] 
        self.edges = [   
                    {'source': 'J_HOME',    'target': 'CU_0',   'motion': 'MoveJ',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'}, 
                    {'source': 'CU_0',      'target': 'J_HOME', 'motion': 'MoveJ',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'}, 
                    {'source': 'CU_0',      'target': 'UNC_0',  'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'}, 
                    {'source': 'UNC_0',     'target': 'CU_0',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'}, 
                    {'source': 'UNC_0',     'target': 'UNC_1',  'motion': 'MoveL',      'speed': self.v_uncp,   'zone': 'fine',    'wait': '0.5'}, 
                    {'source': 'UNC_1',     'target': 'UNC_2',  'motion': 'MoveL',      'speed': self.v_uncp,   'zone': 'fine',    'wait': '0.5'}, 
                    {'source': 'UNC_2',     'target': 'UNC_3',  'motion': 'MoveL',      'speed': self.v_uncp,   'zone': 'fine',    'wait': '0.5'}, 
                    {'source': 'UNC_3',     'target': 'UNC_4',  'motion': 'MoveL',      'speed': self.v_uncp,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'UNC_4',     'target': 'UNC_5',  'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'UNC_5',     'target': 'UNC_0',  'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},

                    {'source': 'J_HOME',    'target': 'H_0',    'motion': 'MoveJ',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'H_0',       'target': 'J_HOME', 'motion': 'MoveJ',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},  
                    {'source': 'CU_0',    'target': 'CL_0',   'motion': 'MoveL',      'speed': self.v_gnrl,     'zone': 'fine',  'wait': '0.5'},
                    {'source': 'CL_0',      'target': 'CU_0', 'motion': 'MoveL',      'speed': self.v_gnrl,     'zone': 'fine',  'wait': '0'}, 

                    {'source': 'CU_0',      'target': 'CU_1',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CU_1',      'target': 'CU_0',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CU_0',      'target': 'CU_2',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CU_2',      'target': 'CU_0',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CU_0',      'target': 'CU_3',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CU_3',      'target': 'CU_0',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CU_0',      'target': 'CU_4',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CU_4',      'target': 'CU_0',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CU_0',      'target': 'CU_5',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CU_5',      'target': 'CU_0',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CU_0',      'target': 'CU_6',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CU_6',      'target': 'CU_0',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},

                    {'source': 'CL_0',      'target': 'CL_1',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CL_1',      'target': 'CL_0',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CL_0',      'target': 'CL_2',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CL_2',      'target': 'CL_0',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CL_0',      'target': 'CL_3',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CL_3',      'target': 'CL_0',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CL_0',      'target': 'CL_4',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CL_4',      'target': 'CL_0',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CL_0',      'target': 'CL_5',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CL_5',      'target': 'CL_0',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CL_0',      'target': 'CL_6',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},
                    {'source': 'CL_6',      'target': 'CL_0',   'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0.5'},

                    {'source': 'CU_1',      'target': 'CU_1F_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_1',      'target': 'CU_1B_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_1F_U',   'target': 'CU_1',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_1B_U',   'target': 'CU_1',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_2',      'target': 'CU_2F_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_2',      'target': 'CU_2B_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_2F_U',   'target': 'CU_2',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_2B_U',   'target': 'CU_2',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_3',      'target': 'CU_3F_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_3',      'target': 'CU_3B_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_3F_U',   'target': 'CU_3',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_3B_U',   'target': 'CU_3',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_4',      'target': 'CU_4F_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_4',      'target': 'CU_4B_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_4F_U',   'target': 'CU_4',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_4B_U',   'target': 'CU_4',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_5',      'target': 'CU_5F_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_5',      'target': 'CU_5B_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_5F_U',   'target': 'CU_5',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_5B_U',   'target': 'CU_5',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_6',      'target': 'CU_6F_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_6',      'target': 'CU_6B_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_6F_U',   'target': 'CU_6',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CU_6B_U',   'target': 'CU_6',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},

                    {'source': 'CU_1F_U',   'target': 'CU_1F_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_1F_D',   'target': 'CU_1F_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_1B_U',   'target': 'CU_1B_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_1B_D',   'target': 'CU_1B_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_2F_U',   'target': 'CU_2F_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_2F_D',   'target': 'CU_2F_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_2B_U',   'target': 'CU_2B_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_2B_D',   'target': 'CU_2B_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_3F_U',   'target': 'CU_3F_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_3F_D',   'target': 'CU_3F_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_3B_U',   'target': 'CU_3B_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_3B_D',   'target': 'CU_3B_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_4F_U',   'target': 'CU_4F_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_4F_D',   'target': 'CU_4F_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_4B_U',   'target': 'CU_4B_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_4B_D',   'target': 'CU_4B_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_5F_U',   'target': 'CU_5F_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_5F_D',   'target': 'CU_5F_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_5B_U',   'target': 'CU_5B_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_5B_D',   'target': 'CU_5B_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_6F_U',   'target': 'CU_6F_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_6F_D',   'target': 'CU_6F_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_6B_U',   'target': 'CU_6B_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CU_6B_D',   'target': 'CU_6B_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},

                    {'source': 'CL_1',      'target': 'CL_1F_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_1',      'target': 'CL_1B_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_1F_U',   'target': 'CL_1',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_1B_U',   'target': 'CL_1',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_2',      'target': 'CL_2F_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_2',      'target': 'CL_2B_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_2F_U',   'target': 'CL_2',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_2B_U',   'target': 'CL_2',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_3',      'target': 'CL_3F_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_3',      'target': 'CL_3B_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_3F_U',   'target': 'CL_3',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_3B_U',   'target': 'CL_3',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_4',      'target': 'CL_4F_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_4',      'target': 'CL_4B_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_4F_U',   'target': 'CL_4',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_4B_U',   'target': 'CL_4',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_5',      'target': 'CL_5F_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_5',      'target': 'CL_5B_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_5F_U',   'target': 'CL_5',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_5B_U',   'target': 'CL_5',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_6',      'target': 'CL_6F_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_6',      'target': 'CL_6B_U',    'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_6F_U',   'target': 'CL_6',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},
                    {'source': 'CL_6B_U',   'target': 'CL_6',       'motion': 'MoveL',      'speed': self.v_gnrl,   'zone': 'fine',    'wait': '0'},

                    {'source': 'CL_1F_U',   'target': 'CL_1F_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_1F_D',   'target': 'CL_1F_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_1B_U',   'target': 'CL_1B_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_1B_D',   'target': 'CL_1B_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_2F_U',   'target': 'CL_2F_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_2F_D',   'target': 'CL_2F_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_2B_U',   'target': 'CL_2B_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_2B_D',   'target': 'CL_2B_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_3F_U',   'target': 'CL_3F_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_3F_D',   'target': 'CL_3F_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_3B_U',   'target': 'CL_3B_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_3B_D',   'target': 'CL_3B_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_4F_U',   'target': 'CL_4F_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_4F_D',   'target': 'CL_4F_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_4B_U',   'target': 'CL_4B_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_4B_D',   'target': 'CL_4B_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_5F_U',   'target': 'CL_5F_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_5F_D',   'target': 'CL_5F_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_5B_U',   'target': 'CL_5B_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_5B_D',   'target': 'CL_5B_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_6F_U',   'target': 'CL_6F_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_6F_D',   'target': 'CL_6F_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_6B_U',   'target': 'CL_6B_D',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                    {'source': 'CL_6B_D',   'target': 'CL_6B_U',       'motion': 'MoveL',      'speed': self.v_gnrl,    'zone': 'fine',     'wait': '0'},
                
                    {'source': 'H_0',     'target': 'H_1',   'motion': 'MoveL',      'speed': self.v_pour,  'zone': 'z1',   'wait': '0'}, 
                    {'source': 'H_1',     'target': 'H_0',   'motion': 'MoveJ',      'speed': self.v_pour,  'zone': 'z1',   'wait': '0'}, 

                    {'source': 'H_1',     'target': 'H_2',   'motion': 'MoveL',      'speed': self.v_pour,  'zone': 'z1',   'wait': '0'}, 
                    {'source': 'H_2',     'target': 'H_1',   'motion': 'MoveL',      'speed': self.v_pour,  'zone': 'z1',   'wait': '0'}, 

                    {'source': 'H_2',     'target': 'H_3',   'motion': 'MoveJ',      'speed': self.v_pour,  'zone': 'z1',   'wait': '0'}, 
                    {'source': 'H_3',     'target': 'H_2',   'motion': 'MoveJ',      'speed': self.v_pour,  'zone': 'z1',   'wait': '0'},

                    {'source': 'H_3',     'target': 'H_4',   'motion': 'MoveJ',      'speed': self.v_pour,  'zone': 'z1',   'wait': '0'}, 
                    {'source': 'H_4',     'target': 'H_3',   'motion': 'MoveJ',      'speed': self.v_pour,  'zone': 'z1',   'wait': '0'} 
                ]
        self.g = ig.Graph.DictList(self.vertices, self.edges,directed=True)
        self.g["title"] = "Waypoint Network"
        print(self.g)

        if self.is_multi_move:
            print("Multimove is active.")
            # get the current joint angles of the second robot. 
            # it will be jogged to current angles for every command for robot 1
            q1, self.q2 = self._get_current_joint_angles()
            print("Robot 1 joint angles: " + str(q1))
            print("Robot 2 joint angles: " + str(self.q2))

        # self._plot_waypoints_network_graph()

        # self._generate_motion_primitive_code("J_HOME","CU_6B_D")

        # self._get_current_joint_angles()

        # self._get_current_pose()

        # self._get_nearest_two_waypoint_name()

        # self._jog_to_nearest_waypoint()

        # self.go_home()

        # self.jog_to("CU_6B_D")

        # self.jog_to("H_3")

    def _plot_waypoints_network_graph(self):
        # Plot in matplotlib
        # Note that attributes can be set globally (e.g. vertex_size), or set individually using arrays (e.g. vertex_color)
        fig, ax = plt.subplots(figsize=(45,45))
        ig.plot(
            self.g,
            target=ax,
            layout="tree", # print nodes in a circular layout # kk, circle,  tree, fr
            vertex_size=0.85,
            vertex_shape="circle",
            vertex_color="white" ,
            vertex_frame_width=0.4,
            # vertex_frame_color="black",
            vertex_label=self.g.vs["name"],
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

    def _generate_motion_primitive_code(self,start,target):
        # Create the motion primitive code for shorthest path between "start" and "target"
        # returns the commands as list of strings 
        motion_code = []
        motion_code_simple = [] # simplifies version of commands to print

        path_v = self.g.get_shortest_paths(start,to=target, output="vpath")[0][1:]
        # print(path_v)
        path_e = self.g.get_shortest_paths(start,to=target, output="epath")[0]
        # print(path_e)
        
        for v,e in zip(path_v,path_e):
            robtarget_str = self._robtarget_str(self.g.vs[v]["name"])
            cmd = "mp." + self.g.es[e]["motion"] + "(" + robtarget_str + "," + self.g.es[e]["speed"] + "," + self.g.es[e]["zone"] + ")"
            motion_code.append(cmd)

            cmd_simple = "mp." + self.g.es[e]["motion"] + "(" + self.g.vs[v]["name"] + "," + self.g.es[e]["speed"] + "," + self.g.es[e]["zone"] + ")"
            motion_code_simple.append(cmd_simple)

            if self.g.es[e]["wait"] != "0":
                cmd = "mp.WaitTime(" + self.g.es[e]["wait"] + ")"
                motion_code.append(cmd)

                motion_code_simple.append(cmd)
        
        # Print motion primitive code for debug
        print("Motion code (simplified) from: " + start + " to: " + target + ":")
        for cmd_simple in motion_code_simple:
            print(cmd_simple)
        return motion_code

    def _get_current_joint_angles(self):
        # Calculate Machine precision for minimum wait time
        eps = 7./3 - 4./3 -1

        if self.is_multi_move:
            # Create a new motion program
            mp = MotionProgram(tool=self.tool0)
            mp2 = MotionProgram()

            mp.WaitTime(eps)
            mp2.WaitTime(eps)

            log_results = self.mot_prog_client.execute_multimove_motion_program([mp,mp2])

            # OLD LOG_RESULTS
            # convert to string and use in memory
            log_results_str = log_results.decode('ascii')
            # print(log_results_str)

            current_joints_str = log_results_str.splitlines(keepends=False)[-1].split(',')[2:2+6]
            current_joints = [float(x) for x in current_joints_str]
            # print("current_joint angles rob1: " + str(current_joints) + " deg.")
            
            current_joints_str2 = log_results_str.splitlines(keepends=False)[-1].split(',')[-6:]
            current_joints2 = [float(x) for x in current_joints_str2]
            # print("current_joint angles rob2: " + str(current_joints2) + " deg.")

            return np.array(current_joints),np.array(current_joints2)  # deg (rob1 angles, rob2 angles)

            # NEW LOG_RESULTS
            current_joints = log_results[-1,2:2+6] 
            current_joints2 = log_results[-1,-6:]
            # print("current_joint angles rob1: " + str(current_joints) + " deg.")
            # print("current_joint angles rob2: " + str(current_joints2) + " deg.")
            return current_joints, current_joints2

        else:
            # Create a new motion program
            mp = MotionProgram(tool=self.tool0)

            mp.WaitTime(eps)
            log_results = self.mot_prog_client.execute_motion_program(mp)

            # OLD LOG_RESULTS
            # convert to string and use in memory
            log_results_str = log_results.decode('ascii')
            # print(log_results_str)

            # current_joints_str = log_results_str.splitlines(keepends=False)[-1].split(',')[-6:]
            # current_joints_str = log_results_str.splitlines(keepends=False)[-1].split(',')[-12:-6]
            current_joints_str = log_results_str.splitlines(keepends=False)[-1].split(',')[2:8]
            current_joints = [float(x) for x in current_joints_str]
            # print("current_joint angles: " + str(current_joints) + " deg.")

            return np.array(current_joints) # deg

            # NEW LOG_RESULTS
            current_joints = log_results[-1,2:2+6]
            # print("current_joint angles rob1: " + str(current_joints) + " deg.")
            return current_joints

    def _get_current_pose(self):
        # calculate current pose from current joint angles
        if self.is_multi_move:
            q,q2 = self._get_current_joint_angles() # deg
        else:
            q = self._get_current_joint_angles() # deg
        q = np.deg2rad(q) # rad

        T = self.robot_rox.fwd(q) # transform
        # print("Rot mat: " + str(T.R))
        # print("Rot RPY: " + str(R2rpy(T.R)))
        # print("Rot quater: " + str(R2q(T.R)))
        # print("Current pose: Position vect=" + str(T.p) + ", Rot quater(w,x,y,z)=" + str(R2q(T.R)))
        return T

    def _get_nearest_two_waypoint_name(self):
        # returns the names of two nearest waypoints as list strings
        # the first one is the nearest, the second one is the second nearest
        first = None
        first_dist = None
        second = None
        second_dist = None

        # # calculate current pose from current joint angles
        # cur_q = self._get_current_joint_angles() # deg
        # cur_q = np.deg2rad(cur_q) # rad

        cur_p = self._get_current_pose().p
        # print("cur_p: " + str(cur_p))

        # compare the current pose with the defined waypoint poses
        for wayp_name, wayp_p, wayp_quat in zip(self.g.vs["name"],self.g.vs["pos"],self.g.vs["quat"]):
            # print(wayp_name, wayp_p, wayp_quat)
            
            # wayp_q = self.robot_rox.inv(wayp_p,R=q2R(wayp_quat),last_joints=cur_q)

            # # find distance with currrent joint angle angle and waypoint joint angle
            # wayp_dist = np.linalg.norm(wayp_q-cur_q)

            # find distance with currrent position and waypoint position
            wayp_dist = np.linalg.norm(wayp_p-cur_p)

            # print(wayp_name,wayp_dist)

            if first is None:
                first = wayp_name
                first_dist = wayp_dist
            else:
                # compare the wayp with first one
                if wayp_dist < first_dist:
                    second = first
                    second_dist = first_dist
                    first = wayp_name
                    first_dist = wayp_dist
                elif second is None or second_dist > wayp_dist:
                    second = wayp_name
                    second_dist = wayp_dist
        
        # print("1st closest:" + first + " with distance: " + str(first_dist))
        # print("2nd closest:" + second + " with distance: " + str(second_dist))

        return [first,second]

    def _jog_to_nearest_waypoint(self):
        # Jogs robot to the nearest waypoint to the current position 
        # Movement is in joint space, hence, USE WITH CAUTION!
        # Returns the name of the jogged nearest waypoint
        nearest_wayp = self._get_nearest_two_waypoint_name()[0] # string
        print("nearest_wayp: " + nearest_wayp)
        mp = MotionProgram(tool=self.tool0)
        mp.MoveJ(eval(self._robtarget_str(nearest_wayp)),v100,fine)

        # print(mp.get_program_rapid())
        log_results = self._motion_program_executer(mp)
        return nearest_wayp

    def _robtarget_str(self,target):
        # generates the robtarget string to be evaluated from the graph given target name
        v = self.g.vs.find(target) # vertex object from the graph
        robtarget_str = "robtarget("+str(v["pos"])+","+str(v["quat"])+",confdata"+str(v["conf"])+",[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])"
        # robtarget([2123.7,-295.974,558.7],[0.707106781,0.0,0.707106781,0.0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
        return robtarget_str

    def _motion_program_executer(self,mp):
        # Takes single robot motion program, executes it according to multi_move setup
        if self.is_multi_move:
            # print(mp.get_program_rapid())
            cmd=mp.get_program_rapid()
            chunks = cmd.split(';')

            mp2 = MotionProgram()
            for line in chunks:
                # print(line)
                if 'Move' in line: # need to send move command to the second robot as well 
                    if '[TRUE,' in line: # if waypoint is a fine point, send fine point command
                        mp2.MoveAbsJ(jointtarget(self.q2,[0]*6),v100,fine)
                    elif '[FALSE,' in line: # if waypoint is not a fine point, send zone command
                        mp2.MoveAbsJ(jointtarget(self.q2,[0]*6),v100,z10)
                elif 'WaitTime' in line: # need to send wait command to the second robot as well
                    mp2.WaitTime(0.1)
            log_results = self.mot_prog_client.execute_multimove_motion_program([mp,mp2])

        else:
            # print(mp.get_program_rapid())
            log_results = self.mot_prog_client.execute_motion_program(mp)
        
        return log_results

    def jog_to(self,target):
        # Jogs robot to nearest waypoint in joint space
        nearest_wayp = self._jog_to_nearest_waypoint()
        # Then jogs robot to desired target point with the waypoints
        motion_code = self._generate_motion_primitive_code(nearest_wayp,target)
        if len(motion_code) > 0:
            mp = MotionProgram(tool=self.tool0)
            for cmd in motion_code:
                eval(cmd)
            # Execute the generated motion code
            # print(mp.get_program_rapid())
            log_results = self._motion_program_executer(mp)

    def go2Home(self):
        target = "J_HOME" 
        self.jog_to(target)

    def go2CabinetUpper(self):
        target = "CU_0" 
        self.jog_to(target)

    def go2Cup(self, place):
        # place is integer btw. 0-11 for cup locations
        # odd numbers are at the back row
        # even numbers are at the front row
        places_dict = {10: "CU_1F_D",
                        11: "CU_1B_D",
                        8: "CU_2F_D",
                        9: "CU_2B_D",
                        6: "CU_3F_D",
                        7: "CU_3B_D",
                        4: "CU_4F_D",
                        5: "CU_4B_D",
                        2: "CU_5F_D",
                        3: "CU_5B_D",
                        0: "CU_6F_D",
                        1: "CU_6B_D"}

        target = places_dict[place]
        self.jog_to(target)

    def go2LidRemover(self):
        target = "UNC_0" 
        self.jog_to(target)

    def RemoveCupLid(self):
        target = "UNC_5" 
        self.jog_to(target)

    def go2Hopper(self):
        target = "H_0" 
        self.jog_to(target)

    def PourMaterial(self):
        target = "H_4" 
        self.jog_to(target)

    def go2CabinetLower(self):
        target = "CL_0" 
        self.jog_to(target)

    def go2Place(self, place):
        # place is integer btw. 0-11 for cup locations
        # odd numbers are at the back row
        # even numbers are at the front row
        places_dict = {10: "CL_1F_D",
                        11: "CL_1B_D",
                        8: "CL_2F_D",
                        9: "CL_2B_D",
                        6: "CL_3F_D",
                        7: "CL_3B_D",
                        4: "CL_4F_D",
                        5: "CL_4B_D",
                        2: "CL_5F_D",
                        3: "CL_5B_D",
                        0: "CL_6F_D",
                        1: "CL_6B_D"}

        target = places_dict[place]
        self.jog_to(target)
        


def main():
    port_num = 9006
    robot_url = 'http://192.168.55.1:80' # ABB computer's URL
    parameter_file = "./robot_request_service.csv" # NOT used for this service
    
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.robot_request_service", port_num) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.robot_request_service")
        # RRN.RequestTimeout = 1 # seconds

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