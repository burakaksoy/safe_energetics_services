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


class DepthPointSelector:
    def __init__(self, ros_image_subscriber):
        self.points = [] # [[x,y,depth(mm)],...,[x,y,depth(mm)]]
        self.window_name = "image"
        self.window_size = (640,480) # default
        self.ros_image_subscriber = ros_image_subscriber

        self.depth_array = None
        self.depth_colormap = None

        self.max_dist_on_color_map = 1500 # mm
        self.desired_alpha = 255./self.max_dist_on_color_map

    def select_points(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.window_size[0], self.window_size[1])

        self.depth_array = self.ros_image_subscriber.get_latest_image()

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_array, alpha=self.desired_alpha), cv2.COLORMAP_JET)
        # displaying the image
        cv2.imshow(self.window_name, self.depth_colormap)

        # setting mouse handler for the image
        # and calling the click_event() function
        cv2.setMouseCallback(self.window_name, self.click_event)

        # wait for a key to be pressed to exit
        cv2.waitKey(0)

        # close the window
        cv2.destroyAllWindows()
        
        return self.points

    # function to display the coordinates of
    # of the points clicked on the image
    def click_event(self, event, x, y, flags, params):
        # checking for left mouse clicks
        if event == cv2.EVENT_LBUTTONDOWN:
            # displaying the coordinates and the depth
            print( "(" + str(x) + "," + str(y) + "), " + str(self.depth_array[y,x]) + " mm")

            # displaying the coordinates
            # on the image window
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(self.depth_colormap, str(self.depth_array[y,x]) + " mm", (x,y), font,
                        1, (0, 0, 0), 2)
            cv2.circle(self.depth_colormap, (x,y), radius=2, color=(0, 0, 0), thickness=-1)
            cv2.imshow('image', self.depth_colormap)

            # Append point to points
            self.points.append([x,y,self.depth_array[y,x]])

# # driver function
# if __name__=="__main__":
#     file_name = "./depth_points_upper.csv"
#     ros_topic = "/depth_to_rgb/hw_registered/image_rect_raw" #RGB version: "/rgb/image_rect_color" 
    
#     ros_image_subscriber = ROS_image_subscriber(ros_topic)
#     point_selector = DepthPointSelector(ros_image_subscriber)

#     depth_points = point_selector.select_points()
#     # print(depth_points)
    
#     # Create a csv with depth points
#     df = pd.DataFrame(depth_points, columns =['x', 'y','depth_mm'])
#     # print(df)
#     df.to_csv(file_name, sep=',',index=False)



class CupDetector_impl():
    def __init__(self):
        self.file_name = "./depth_points_upper.csv"
        self.ros_topic = "/depth_to_rgb/hw_registered/image_rect_raw" #RGB version: "/rgb/image_rect_color" 
        self.depth_tolerance = 15.0 # mm

        # read csv file
        self.file_name = os.path.expanduser(self.file_name)
        self.df = pd.read_csv(self.file_name)
        self.depth_points = self.df.values.tolist()
        
        # Read image from ros topic
        self.ros_image_subscriber = ROS_image_subscriber(self.ros_topic)
        self.depth_array = None
        
        self.cup_places = []
        self.cup_places_wrong = []

        self.num_of_wrong_cups = len(self.depth_points)
        # self.num_of_cups = 0


    def places_of_cups(self):
        self.cup_places = []
        self.num_of_cups()
        return self.cup_places

    def places_of_wrong_cups(self):
        self.cup_places_wrong = []
        self.num_of_cups()
        return self.cup_places_wrong

    def num_of_wrong_cups(self):
        pass

    # Returns the number of cups in the image
    def num_of_cups(self):
        # Read image from ros topic
        self.depth_array = self.ros_image_subscriber.get_latest_image()

        num_of_cups = 0
        for point in self.depth_points:
            x = point[0]
            y = point[1]
            desired_depth = point[2]

            max_depth = (desired_depth + self.depth_tolerance)
            min_depth = (desired_depth - self.depth_tolerance)
            if (self.depth_array[y,x] <= max_depth) and (self.depth_array[y,x] >= min_depth):
                num_of_cups = num_of_cups + 1
                self.cup_places.append(str((x,y)))
            else:
                print("(" + str((x,y))+ ") could not be found at desired depth: " + str(desired_depth) + "mm. Depth is:" + str(self.depth_array[y,x]) + " mm.\n")
                self.cup_places_wrong.append(str((x,y)))

        return num_of_cups

def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.rr_cup_detector_service", 9001) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.cup_detector")

        # create object
        CupDetector_inst = CupDetector_impl()
        # register service with service name "CupDetector", type "experimental.pluginCameraCalibration.CupDetector", actual object: CupDetector_inst
        RRN.RegisterService("CupDetector","experimental.cup_detector.CupDetector",CupDetector_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("cup_detector Server started, press enter to quit...\n")
        else:
            raw_input("cup_detector Server started, press enter to quit...")

if __name__ == '__main__':
    main()