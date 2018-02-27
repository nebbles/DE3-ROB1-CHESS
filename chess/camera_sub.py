from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import message_filters
from message_filters import Subscriber
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import argparse

import cv2

#dont forget to roslaunch openni2_launch openni2.launch
#roscore
 

class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("/camera/rgb/image_rect_color",Image)

    self.bridge = CvBridge()
    image_sub = Subscriber("/camera/rgb/image_rect_color",Image)
    depth_sub = Subscriber("/camera/depth_registered/image_raw", Image)

    tss = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub],queue_size=10, slop=0.5)
    # self.callback_d(image_sub,depth_sub)                                    
    tss.registerCallback(self.callback)
    print('init')

  def callback(self,img, depth):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
    except CvBridgeError as e:
      print(e)
    try:
      depth_image_raw = self.bridge.imgmsg_to_cv2(depth, "passthrough")
      depth_image = ((255*depth_image_raw)).astype(np.uint8)
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Image window", cv_image)
    cv2.imshow("Depth window", depth_image)
    cv2.waitKey(1)

    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
  
