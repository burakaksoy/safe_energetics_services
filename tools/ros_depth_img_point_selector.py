import rospy

import sensor_msgs.msg
# import kinova_msgs.msg

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import argparse
import sys
import pandas as pd
import math
import os
import time


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

        self.depth_array = ros_image_subscriber.get_latest_image()

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
            # x = 912
            # y = 1312

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

# driver function
if __name__=="__main__":
    file_name = "./depth_points_lid_off.csv"
    ros_topic = "/depth_to_rgb/hw_registered/image_rect_raw" #RGB version: "/rgb/image_rect_color" 
    
    ros_image_subscriber = ROS_image_subscriber(ros_topic)
    point_selector = DepthPointSelector(ros_image_subscriber)

    depth_points = point_selector.select_points()
    # print(depth_points)
    
    # Create a csv with depth points
    df = pd.DataFrame(depth_points, columns =['x', 'y','depth_mm'])
    # print(df)
    df.to_csv(file_name, sep=',',index=False)
        
    