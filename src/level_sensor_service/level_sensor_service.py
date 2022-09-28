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


class LevelSensor_impl():
    def __init__(self,parameter_file,ros_topic, depth_tolerance):
        self.file_name = parameter_file
        self.ros_topic = ros_topic 
        self.depth_tolerance = depth_tolerance

        # read csv file
        self.file_name = os.path.expanduser(self.file_name)
        self.df = pd.read_csv(self.file_name)
        self.params = self.df.values.tolist()

        # Parse the parameters
        self.roi_x = self.params[0][0]
        self.roi_y = self.params[0][1]
        self.roi_w = self.params[0][2]
        self.roi_h = self.params[0][3]
        self.low_level = self.params[0][4] # mm
        self.high_level = self.params[0][5] # mm

        print("roi_x: " + str(self.roi_x))
        print("roi_y: " + str(self.roi_y))
        print("roi_w: " + str(self.roi_w))
        print("roi_h: " + str(self.roi_h))
        print("low_level: " + str(self.low_level))
        print("high_level: " + str(self.high_level))
        
        # Read image from ros topic
        self.ros_image_subscriber = ROS_image_subscriber(self.ros_topic)


    # Returns ...
    def isLevelLow(self):
        # # Read image from ros topic
        # num_readings = 10
        # levels = np.zeros(1,num_readings)
        # for i in range(num_readings):
        #     depth_array = self.ros_image_subscriber.get_latest_image()
        #     # print("depth_array is of type:", type(depth_array))
        #     # print(str(depth_array))
        #     print("")
        #     print("image h,w: " + str(depth_array.shape))

        #     depth_array_roi = depth_array[self.roi_y:self.roi_y+self.roi_h,self.roi_x:self.roi_x+self.roi_w]
        #     # print("depth_array_roi is of type:", type(depth_array_roi))
        #     # print(str(depth_array_roi))
        #     print("ROI image h,w: " + str(depth_array_roi.shape))

        #     avr_level = np.mean(depth_array_roi)
        #     levels[0,i] = avr_level

        # print("levels: " + str(levels))
        # avr_level = np.mean(levels)
        # print("avr_level: "+str(avr_level))
        # print("low_level: "+str(self.low_level))

        # if avr_level >= self.low_level:
            return True
        # else:
            # return False

    # Returns ...
    def isLevelHigh(self):
        # # Read image from ros topic
        # num_readings = 10
        # levels = np.zeros(1,num_readings)
        # for i in range(num_readings):
        #     depth_array = self.ros_image_subscriber.get_latest_image()
        #     # print("depth_array is of type:", type(depth_array))
        #     # print(str(depth_array))
        #     print("")
        #     print("image h,w: " + str(depth_array.shape))

        #     depth_array_roi = depth_array[self.roi_y:self.roi_y+self.roi_h,self.roi_x:self.roi_x+self.roi_w]
        #     # print("depth_array_roi is of type:", type(depth_array_roi))
        #     # print(str(depth_array_roi))
        #     print("ROI image h,w: " + str(depth_array_roi.shape))

        #     avr_level = np.mean(depth_array_roi)
        #     levels[0,i] = avr_level

        # print("levels: " + str(levels))
        # avr_level = np.mean(levels)
        # print("avr_level: "+str(avr_level))
        # print("high_level: "+str(self.high_level))

        # if avr_level <= self.high_level:
            return True
        # else:
            # return False

def main():
    port_num = 9001
    parameter_file = "./level_sensor_service.csv"
    ros_topic = "/depth_to_rgb/hw_registered/image_rect_raw" #RGB version: "/rgb/image_rect_color"
    depth_tolerance = 15 #mm

    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.level_sensor_service", port_num) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.level_sensor_service")

        # create object
        LevelSensor_inst = LevelSensor_impl(parameter_file,ros_topic,depth_tolerance)
        # register service with service name "LevelSensor", type "experimental.level_sensor_service.LevelSensor", actual object: LevelSensor_inst
        RRN.RegisterService("LevelSensor","experimental.level_sensor_service.LevelSensor",LevelSensor_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("level_sensor_service started, press enter to quit...\n")
        else:
            raw_input("level_sensor_service started, press enter to quit...")

if __name__ == '__main__':
    main()