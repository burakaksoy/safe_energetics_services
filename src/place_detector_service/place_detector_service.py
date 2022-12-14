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



class PlaceDetector_impl():
    def __init__(self,parameter_file,ros_topic, depth_tolerance):
        self.file_name = parameter_file
        self.ros_topic = ros_topic 
        self.depth_tolerance = depth_tolerance

        # read csv file
        self.file_name = os.path.expanduser(self.file_name)
        self.df = pd.read_csv(self.file_name)
        self.depth_points = self.df.values.tolist()

        print("Depth tolerance: "+str(self.depth_tolerance))
        print("Place Desired Points: ")
        print(str(self.depth_points))
        
        # Read image from ros topic
        self.ros_image_subscriber = ROS_image_subscriber(self.ros_topic)
        


    def getAvailableLocations(self):
        print("\nNew getAvailableLocations request!")
        print("------------------------------------")

        UNavailable_locations_all = []
        locations_all = [0,1,2,3,4,5,6,7,8,9,10,11]
    
        num = 10
        for i in range(num):
            if i == 0:
                depth_array = self.ros_image_subscriber.get_latest_image()
            else:
                depth_array = depth_array + self.ros_image_subscriber.get_latest_image()
        depth_array = depth_array / float(num)

        itr = -1
        for point in self.depth_points:
            print(str(point))
            itr = itr + 1
            x = int(point[0])
            y = int(point[1])
            desired_depth = point[2]

            max_depth = (desired_depth + self.depth_tolerance)
            min_depth = (desired_depth - self.depth_tolerance)
            if (depth_array[y,x] <= max_depth) and (depth_array[y,x] >= min_depth):
                UNavailable_locations_all.append(itr)
                print("(" + str((x,y))+ ") is found at Depth:" + str(depth_array[y,x]) + " mm.\n")
            else:
                print("(" + str((x,y))+ ") could not be found at desired depth: " + str(desired_depth) + "mm. Depth is:" + str(depth_array[y,x]) + " mm.")

        # To get elements which are in locations_all but not in UNavailable_locations_all:
        available_locations_all = list(set(locations_all) - set(UNavailable_locations_all))
        # sort available locations in ascending order
        available_locations_all.sort() 

        print("Available locations before filtering: ")
        print(str(available_locations_all))

        available_locations = []
        while len(available_locations_all) > 0:
            location = available_locations_all.pop(0)
            if (location % 2) == 0:
                if (location + 1) in available_locations_all:
                    idx = available_locations_all.index(location+1)
                    available_locations.append(available_locations_all.pop(idx))
                available_locations.append(location)


        print("Available locations in order: ")
        print(str(available_locations))
        
        return available_locations

def main():
    port_num = 9003
    parameter_file = "./place_detector_service.csv"
    ros_topic = "/depth_to_rgb/hw_registered/image_rect_raw" #RGB version: "/rgb/image_rect_color"
    depth_tolerance = 35 #mm

    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.place_detector_service", port_num) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.place_detector_service")

        # create object
        PlaceDetector_inst = PlaceDetector_impl(parameter_file,ros_topic,depth_tolerance)
        # register service with service name "PlaceDetector", type "experimental.place_detector_service.PlaceDetector", actual object: PlaceDetector_inst
        RRN.RegisterService("PlaceDetector","experimental.place_detector_service.PlaceDetector",PlaceDetector_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("place_detector_service started, press enter to quit...\n")
        else:
            raw_input("place_detector_service started, press enter to quit...")

if __name__ == '__main__':
    main()