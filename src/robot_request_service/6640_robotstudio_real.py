import numpy as np
from general_robotics_toolbox import *
import sys
from abb_motion_program_exec_client import *
# from toolbox.robots_def import abb_irb52
# sys.path.append('./toolbox')
# from robots_def import *
# from error_check import *
# from MotionSend import *
# from utils import *

CL_0 = robtarget([2011.1,-500,1095.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_1 = robtarget([2061.1,-238.062,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_1B_D = robtarget([2336.513,-238.062,1075.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_1B_U = robtarget([2336.513,-238.062,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_1F_D = robtarget([2263.488,-238.062,1075.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_1F_U = robtarget([2263.488,-238.062,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_2 = robtarget([2061.1,-342.837,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_2B_D = robtarget([2336.513,-342.837,1075.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_2B_U = robtarget([2336.513,-342.837,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_2F_D = robtarget([2263.488,-342.837,1075.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_2F_U = robtarget([2263.488,-342.837,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_3 = robtarget([2061.1,-447.612,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_3B_D = robtarget([2336.513,-447.612,1075.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_3B_U = robtarget([2336.513,-447.612,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_3F_D = robtarget([2263.488,-447.612,1075.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_3F_U = robtarget([2263.488,-447.612,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_4 = robtarget([2061.1,-552.387,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_4B_D = robtarget([2336.513,-552.387,1075.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_4B_U = robtarget([2336.513,-552.387,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_4F_D = robtarget([2263.488,-552.387,1075.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_4F_U = robtarget([2263.488,-552.387,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_5 = robtarget([2061.1,-657.162,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_5B_D = robtarget([2336.513,-657.162,1075.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_5B_U = robtarget([2336.513,-657.162,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_5F_D = robtarget([2263.488,-657.162,1075.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_5F_U = robtarget([2263.488,-657.162,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_6 = robtarget([2061.1,-761.937,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_6B_D = robtarget([2336.513,-761.937,1075.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_6B_U = robtarget([2336.513,-761.937,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_6F_D = robtarget([2263.488,-761.937,1075.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CL_6F_U = robtarget([2263.488,-761.937,1115.27],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_0 = robtarget([2011.1,-500,1400.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_1 = robtarget([2061.1,-238.062,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_1B_D = robtarget([2336.513,-238.062,1380.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_1B_U = robtarget([2336.513,-238.062,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_1F_D = robtarget([2263.488,-238.062,1380.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_1F_U = robtarget([2263.488,-238.062,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_2 = robtarget([2061.1,-342.837,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_2B_D = robtarget([2336.513,-342.837,1380.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_2B_U = robtarget([2336.513,-342.837,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_2F_D = robtarget([2263.488,-342.837,1380.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_2F_U = robtarget([2263.488,-342.837,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_3 = robtarget([2061.1,-447.612,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_3B_D = robtarget([2336.513,-447.612,1380.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_3B_U = robtarget([2336.513,-447.612,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_3F_D = robtarget([2263.488,-447.612,1380.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_3F_U = robtarget([2263.488,-447.612,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_4 = robtarget([2061.1,-552.387,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_4B_D = robtarget([2336.513,-552.387,1380.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_4B_U = robtarget([2336.513,-552.387,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_4F_D = robtarget([2263.488,-552.387,1380.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_4F_U = robtarget([2263.488,-552.387,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_5 = robtarget([2061.1,-657.162,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_5B_D = robtarget([2336.513,-657.162,1380.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_5B_U = robtarget([2336.513,-657.162,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_5F_D = robtarget([2263.488,-657.162,1380.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_5F_U = robtarget([2263.488,-657.162,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_6 = robtarget([2061.1,-761.937,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_6B_D = robtarget([2336.513,-761.937,1380.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_6B_U = robtarget([2336.513,-761.937,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_6F_D = robtarget([2263.488,-761.937,1380.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
CU_6F_U = robtarget([2263.488,-761.937,1420.07],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
H_0 = robtarget([1334.925,-1100,2006.6],[0.5,0.5,0.5,-0.5],confdata(-1,-2,1,0),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
H_1 = robtarget([1334.925,-1400,2006.6],[0.5,0.5,0.5,-0.5],confdata(-1,-2,1,0),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
H_2 = robtarget([1434.925,-1400,2106.6],[0.5,0.5,0.5,-0.5],confdata(-1,-1,0,0),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
H_3 = robtarget([1459.925,-1400,2106.6],[0.5,0.5,-0.5,0.5],confdata(-1,-1,-2,0),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
UNC_0 = robtarget([2135.7,-395.974,572.2],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
UNC_1 = robtarget([2135.7,-295.974,578.7],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
UNC_2 = robtarget([2123.7,-295.974,578.7],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])
UNC_3 = robtarget([2123.7,-295.974,558.7],[0.707106781,0,0.707106781,0],confdata(-1,-1,0,1),[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09])

def main():
    # Define robot
    # robot = abb6640()

    J_HOME = jointtarget([0,0,0,0,0,0],[0]*6)

    tool0 = tooldata(True,pose([5.5,0,270.7],[1,0,0,0]),loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0))   
    mp = MotionProgram(tool=tool0)
    
    # Go home initially
    mp.MoveAbsJ(J_HOME,v100,fine)
    mp.WaitTime(2)
    
    
    # # Approach to cabinet upper layer
    # mp.MoveJ(CU_0,v500,fine)

    # # Look for available cups
    # # TODO
    # mp.WaitTime(1)

    # # Approach to available cup colum (eg 6)
    # mp.MoveL(CU_6,v500,fine)
    # mp.WaitTime(0.5)

    # # Approach to cup inside cabinet
    # mp.MoveL(CU_6B_U,v500,fine)
    # mp.MoveL(CU_6B_D,v500,fine)

    # # Grasp the cup
    # # TODO
    # mp.WaitTime(1)

    # # # Carry the cup outside cabinet
    # # mp.MoveL(CU_6B_U,v500,fine)
    # # mp.MoveL(CU_6,v500,fine)
    # mp.MoveJ(CU_0,v500,fine)
    # mp.WaitTime(1)
    

    # # Look for cup is successfully removed from cabinet
    # # TODO

    # # Go to uncapping mechanism
    # mp.MoveL(UNC_0,v500,fine)
    # mp.WaitTime(0.5)
    # mp.MoveL(UNC_1,v100,fine)
    # mp.WaitTime(0.5)
    # mp.MoveL(UNC_2,v100,fine)
    # mp.WaitTime(0.5)
    # mp.MoveL(UNC_3,v100,fine)
    # mp.WaitTime(0.5)
    # mp.MoveL(UNC_0,v100,fine)
    # mp.WaitTime(0.5)

    # # Make sure the cup is removed, accordingly re-try uncapping
    # # TODO
    # mp.WaitTime(1)

    # # Go back to in front of upper layer linearly
    # mp.MoveL(CU_0,v500,fine)

    # # Return to Home
    # mp.MoveAbsJ(J_HOME,v500,fine)

    # # Go to Hopper
    # mp.MoveJ(H_0,v500,fine)

    # # Do pouring
    # mp.MoveL(H_1,v500,fine)
    # mp.MoveL(H_2,v500,fine)
    # mp.MoveJ(H_3,v500,fine)
    # mp.WaitTime(1.5)

    # # Revert the pouring
    # mp.MoveL(H_2,v500,fine)
    # mp.MoveL(H_1,v500,fine)

    # # Return to Home
    # mp.MoveJ(H_0,v500,fine)
    # mp.MoveAbsJ(J_HOME,v500,fine)

    # # Go to lower side of the cabinet to return the empty cup
    # mp.MoveJ(CL_0,v500,fine)

    # # Look for a place available to put the cup
    # # TODO
    # mp.WaitTime(1)

    # # Approach to available cup placing colum (eg 6)
    # mp.MoveL(CL_6,v500,fine)
    # mp.WaitTime(0.5)

    # # Approach to cup place inside cabinet
    # mp.MoveL(CL_6B_U,v500,fine)
    # mp.MoveL(CL_6B_D,v500,fine)

    # # Relaease the cup
    # # TODO
    # mp.WaitTime(1)

    # # Carry the robot outside cabinet
    # mp.MoveL(CL_6B_U,v500,fine)
    # mp.MoveL(CL_6,v500,fine)
    # mp.MoveJ(CL_0,v500,fine)
    
    # # Look for cup is successfully placed to cabinet
    # # TODO
    # mp.WaitTime(1)

    # # Return to Home
    # mp.MoveAbsJ(J_HOME,v500,fine)

    print(mp.get_program_rapid())

    client = MotionProgramExecClient(base_url='http://192.168.55.1:80')
    # client = MotionProgramExecClient()
    log_results = client.execute_motion_program(mp)

    # # Write log csv to file
    # with open("log.csv","wb") as f:
    #     f.write(log_results)

    # Or convert to string and use in memory
    log_results_str = log_results.decode('ascii')
    print(log_results_str)

def main2():
    J_HOME = jointtarget([0,0,0,0,0,0],[0]*6)

    tool0 = tooldata(True,pose([5.5,0,270.7],[1,0,0,0]),loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0))   
    mp = MotionProgram(tool=tool0)

    mp2 = MotionProgram(tool=tool0)

    # Go home initially
    mp.MoveAbsJ(J_HOME,v100,fine)
    mp.WaitTime(2)
    mp2.MoveAbsJ(J_HOME,v100,fine)
    mp2.WaitTime(2)

    print(mp.get_program_rapid())

    client = MotionProgramExecClient(base_url='http://192.168.55.1:80')
    # client = MotionProgramExecClient()
    log_results = client.execute_motion_program_multimove([mp,mp2])
    log_results = client.execute_motion_program_multimove([mp3,mp2])

if __name__ == "__main__":
    main2()