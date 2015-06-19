#!/usr/bin/env python
__author__ = 'chin'

DEBUGPLOT = False

import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class Kinect2():

    def __init__(self, name):
        """
        Function to initiate the class. Called when creating a new instance.
        :param name: the name of the ros node
        """

        rospy.loginfo("Starting node %s" % name)
        self.bridge = CvBridge()        # Creating an OpenCV bridge object to convert an OpenCV image from the ROS image
        self.cv_rgb_img = None
        self.cv_depth_img = None

        self.rgb_sub = rospy.Subscriber(    # Creating a subscriber listening to the kinect2 image topic
            "/kinect2/hd/image_color_rect", # The topic to which it should listen to
            Image,                          # The data type of the topic
            callback=self.rgb_callback,     # The callback function that is triggered when a new message arrives
            queue_size=1                    # Disregard every message but the latest
        )

        self.depth_sub = rospy.Subscriber(  # Creating a subscriber listening to the kinect2 image topic
            "/kinect2/hd/image_depth_rect", # The topic to which it should listen to
            Image,                          # The data type of the topic
            callback=self.depth_callback,   # The callback function that is triggered when a new message arrives
            queue_size=1                    # Disregard every message but the latest
        )

    """
        self.pcl_sub = rospy.Subscriber(    # Creating a subscriber listening to the kinect2 image topic
            "/kinect2/hd/points",           # The topic to which it should listen to
            PointCloud2,                    # The data type of the topic
            callback=self.pcl_callback,     # The callback function that is triggered when a new message arrives
            queue_size=1                    # Disregard every message but the latest
        )

    def pcl_callback(self, pcldata):
        rospy.loginfo("Received point clouds of size: %i x %i" % (pcldata.width, pcldata.height))  # Make sure we receive something
        print pcldata.fields
        print pcldata.point_step, pcldata.row_step
    """

    def rgb_callback(self, img):
        rospy.loginfo("Received RGB image of size: %i x %i" % (img.width, img.height))  # Make sure we receive something

        try:
            self.cv_rgb_img = self.bridge.imgmsg_to_cv2(img, "bgr8")   # Convert to OpenCV image
        except CvBridgeError, e:
            print e

    def depth_callback(self, img):
        rospy.loginfo("Received Depth image of size: %i x %i" % (img.width, img.height))   # Make sure we receive something

        try:
            self.cv_depth_img = self.bridge.imgmsg_to_cv2(img)   # Convert to OpenCV image
        except CvBridgeError, e:
            print e

        if DEBUGPLOT:
            cv2.imshow("RGB image", self.cv_rgb_img)              # Showing the RGB image
            cv2.imshow("Depth image", self.cv_depth_img)          # Showing the depth image
            cv2.waitKey(1)

        """
        # Convert the depth image to a Numpy array since most cv2 functions require Numpy arrays
        depth_array = np.array(depth_img, dtype=np.float16)

        # Normalize the depth image to fall between 0 (black) to 1 (white)
        cv2.normalize(depth_img, self.cv_depth_img, 0, 1, cv2.NORM_MINMAX)
        print self.cv_depth_img
        """
        """
        self.cv_depth_img = np.zeros((img.height, img.width, 3), np.uint8)
        mx = 0
        for i in range(img.width):
            for j in range(img.height):
                d = depth_img[j, i]
                if not(np.isnan(d)) and d > mx:
                    mx = d

        rospy.loginfo("Maximum depth: %i" % mx)   # Make sure we receive something

        for i in range(img.width):
            for j in range(img.height):
                v = depth_img[j, i]/mx*255
                self.cv_depth_img[j, i] = (v, v, v)
        """


# The block below will be executed when the python file is executed
# __name__ and __main__ are built-in python variables and need to start and end with *two* underscores
if __name__ == '__main__':
    rospy.init_node("k2rgbd")        # Create a node of name k2img
    k2 = Kinect2(rospy.get_name)    # Create an instance of above class
    rospy.spin()                    # Function to keep the node running until terminated via Ctrl+C
