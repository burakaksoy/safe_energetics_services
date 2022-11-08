import rospy
import sensor_msgs.msg
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

import argparse
import sys
import pandas as pd
import math
import os
import time

import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 

class ROS_image_subscriber():
    def __init__(self,ros_topic):

        rospy.init_node('ros_image_subscriber', anonymous=True)
    
        # Raw Image topic name that this node subscribes
        self.image_topic_name = rospy.get_param('~image_topic_name', ros_topic)

        self.bridge = CvBridge() # To convert ROS images to openCV imgs.

        self.image = None # Variable to store the latest image
        self.encoding = "passthrough" # image encoding
        # self.encoding = "bgr8" # image encoding
        # self.encoding = "mono16" # image encoding
        self.timeout = 5 # seconds to wait for an image


    def get_latest_image(self):
        success = False
        while (not success) and (not rospy.is_shutdown()):
            try:
                # read the image msg from the topic:
                msg = rospy.wait_for_message(self.image_topic_name, sensor_msgs.msg.Image, timeout=self.timeout)
                # Convert a ROS image message into an cv::Mat, module cv_bridge:
                self.image = self.bridge.imgmsg_to_cv2(msg, self.encoding)

                success = True
            except CvBridgeError as e:
                rospy.logerr(e)

        return self.image


class LidChecker_impl():
    def __init__(self,parameter_file,ros_topic, depth_tolerance):
        self.file_name = parameter_file
        self.ros_topic = ros_topic 
        self.depth_tolerance = depth_tolerance

        # read csv file
        self.file_name = os.path.expanduser(self.file_name)
        self.df = pd.read_csv(self.file_name)
        self.params = self.df.values.tolist()

        # Parse the parameters
        self.x_off = self.params[0][0]
        self.y_off = self.params[0][1]
        self.depth_off = self.params[0][2]

        self.x_on = self.params[1][0]
        self.y_on = self.params[1][1]
        self.depth_on = self.params[1][2]

        self.x_cup = self.params[2][0]
        self.y_cup = self.params[2][1]
        self.depth_cup = self.params[2][2]

        print("Depth tolerance: "+str(self.depth_tolerance))

        print("Lid off parameters:")
        print("x_off: " + str(self.x_off))
        print("y_off: " + str(self.y_off))
        print("depth_off: " + str(self.depth_off))

        print("Lid on parameters:")
        print("x_on: " + str(self.x_on))
        print("y_on: " + str(self.y_on))
        print("depth_on: " + str(self.depth_on))
        
        print("Cup body parameters:")
        print("x_cup: " + str(self.x_cup))
        print("y_cup: " + str(self.y_cup))
        print("depth_cup: " + str(self.depth_cup))

        # Read image from ros topic
        self.ros_image_subscriber = ROS_image_subscriber(self.ros_topic)


    # Returns ...
    def isLidOff(self):
        num_readings = 10
        levels = np.zeros(num_readings,)

        for i in range(num_readings):
            # Read image from ros topic
            depth_array = self.ros_image_subscriber.get_latest_image()
            levels[i] = depth_array[self.y_off,self.x_off]    

        print("")
        print("image h,w: " + str(depth_array.shape))
        print("Levels: " + str(levels))

        avr_level = np.mean(levels)
        print("avr_level: "+str(avr_level))
        print("depth_off_level: " + str(self.depth_off))

        max_depth = (self.depth_off + self.depth_tolerance)
        min_depth = (self.depth_off - self.depth_tolerance)
        if (avr_level <= max_depth) and (avr_level >= min_depth):
            return True
        else:
            return False

    # Returns ...
    def isLidOn(self):
        num_readings = 10
        levels = np.zeros(num_readings,)

        for i in range(num_readings):
            # Read image from ros topic
            depth_array = self.ros_image_subscriber.get_latest_image()
            levels[i] = depth_array[self.y_on,self.x_on]    

        print("")
        print("image h,w: " + str(depth_array.shape))
        print("Levels: " + str(levels))

        avr_level = np.mean(levels)
        print("avr_level: "+str(avr_level))
        print("depth_on_level: " + str(self.depth_on))

        max_depth = (self.depth_on + self.depth_tolerance)
        min_depth = (self.depth_on - self.depth_tolerance)
        if (avr_level <= max_depth) and (avr_level >= min_depth):
            return True
        else:
            return False

    # Returns ...
    def isGraspedCorrectly(self):
        num_readings = 10
        levels = np.zeros(num_readings,)

        for i in range(num_readings):
            # Read image from ros topic
            depth_array = self.ros_image_subscriber.get_latest_image()
            levels[i] = depth_array[self.y_cup,self.x_cup]    

        print("")
        print("image h,w: " + str(depth_array.shape))
        print("Levels: " + str(levels))

        avr_level = np.mean(levels)
        print("avr_level: "+str(avr_level))
        print("depth_cup_level: " + str(self.depth_cup))

        max_depth = (self.depth_cup + self.depth_tolerance)
        min_depth = (self.depth_cup - self.depth_tolerance)
        if (avr_level <= max_depth) and (avr_level >= min_depth):
            return True
        else:
            return False


def main():
    port_num = 9004
    parameter_file = "./lid_checker_service.csv"
    ros_topic = "/depth_to_rgb/hw_registered/image_rect_raw" #RGB version: "/rgb/image_rect_color"
    depth_tolerance = 6 #mm

    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.lid_checker_service", port_num) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.lid_checker_service")

        # create object
        LidChecker_inst = LidChecker_impl(parameter_file,ros_topic,depth_tolerance)
        # register service with service name "LidChecker", type "experimental.lid_checker_service.LidChecker", actual object: LidChecker_inst
        RRN.RegisterService("LidChecker","experimental.lid_checker_service.LidChecker",LidChecker_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("lid_checker_service started, press enter to quit...\n")
        else:
            raw_input("lid_checker_service started, press enter to quit...")

if __name__ == '__main__':
    main()