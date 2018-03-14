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


    self.rgb = None
    self.depth = None

    self.bridge = CvBridge()

    # rospy.init_node('image_converter', anonymous=True)

    image_sub = Subscriber("/camera/rgb/image_rect_color", Image)
    depth_sub = Subscriber("/camera/depth_registered/image_raw", Image)
    
    tss = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub],queue_size=10, slop=0.5)                                   
    tss.registerCallback(self.callback)
    
    if self.debug:
      print('Initialised Cam feed')

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

    if self.debug:
      print("Image has been converted.")

    self.rgb = cv_image
    self.depth = depth_image
