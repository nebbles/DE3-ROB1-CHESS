# MAKE SURE TO RUN: 
# roscore
# roslaunch openni2_launch openni2.launch
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
import multiprocessing as mp


class CameraFeed:
    def __init__(self, debug=False):
        self.debug = debug
        # init node / or not
        rospy.init_node('image_converter', anonymous=True)

        self.rgb = None
        self.depth = None

        self.debug = debug
        self.bridge = CvBridge()

        # rospy.Subscriber('franka_current_position', Float64MultiArray,
        #                  self.subscribe_position_callback, queue_size=1)


        rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.callback, queue_size=1)
        rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.callback, queue_size=1)
        time.sleep(0.5)

        # tss = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], queue_size=10,
        #                                                   slop=0.5)
        # tss.registerCallback(self.callback)

        if self.debug:
            print('Initialised ImageConverter')

    def callback(self, img, depth, debug=False):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            depth_image_raw = self.bridge.imgmsg_to_cv2(depth, "passthrough")
            depth_image = ((255 * depth_image_raw)).astype(np.uint8)
        except CvBridgeError as e:
            print(e)

        self.

    # def get_frames(self):
    #     while self.rgb_q.empty() and self.depth_q.empty():
    #         time.sleep(0.1)
    #
    #     # we fetch latest image from queue
    #     rgbFrame = self.rgb_q.get()
    #     depthFrame = self.depth_q.get()
    #
    #     return rgbFrame, depthFrame


