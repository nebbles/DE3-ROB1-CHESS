#!/usr/bin/env python
# Benedict Greenberg, March 2018
# http://github.com/nebbles
#
# Make sure that you are running:
#   roscore
#   roslaunch openni2_launch openni2.launch
from __future__ import print_function
import roslib
import rospy
import cv2
import message_filters
from message_filters import Subscriber
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np


class CameraFeed:
    def __init__(self, debug=False, init_ros_node=False):
        self.debug = debug

        self.rgb_raw = None
        self.depth_raw = None

        self.bridge = CvBridge()

        if init_ros_node:
            rospy.init_node('image_converter', anonymous=True)

        self.image_sub = Subscriber("/camera/rgb/image_rect_color", Image)
        self.depth_sub = Subscriber("/camera/depth_registered/image_raw", Image)
        self.tss = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub],
                                                               queue_size=10, slop=0.5)
        self.tss.registerCallback(self.callback)

        time.sleep(0.5)

        if self.debug:
            print("Waiting for subscriber to return initial camera feed.")
        while self.depth_raw is None:
            time.sleep(0.1)
        if self.debug:
            print('Camera feed initialisation successful.')

    def callback(self, img, depth):
        self.rgb_raw = img
        self.depth_raw = depth
        if self.debug:
            print("New images have been collected.")

    def get_frames(self):
        cv_image = None
        depth_image = None
        while cv_image is None and depth_image is None:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.rgb_raw, "bgr8")
            except CvBridgeError as e:
                print(e)

            try:
                depth_image_raw = self.bridge.imgmsg_to_cv2(self.depth_raw, "passthrough")
                # noinspection PyRedundantParentheses
                depth_image = ((255 * depth_image_raw)).astype(np.uint8)
            except CvBridgeError as e:
                print(e)

        if self.debug:
            print("Image has been converted.")

        return cv_image, depth_image
