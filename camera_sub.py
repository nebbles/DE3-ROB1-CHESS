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
#ros core
 
#test_image = None
#est_depth = None
                                                                                                                     
class image_converter:

  def __init__(self):

    global cv_image, depth_image
    #self.image_pub = rospy.Publisher("/camera/rgb/image_rect_color",Image)

    self.bridge = CvBridge()
    # self.image = None
    # self.depth = None

    image_sub = Subscriber("/camera/rgb/image_rect_color", Image)
    depth_sub = Subscriber("/camera/depth_registered/image_raw", Image)

    #self.image_pub = rospy.Publisher("/image", Image, queue_size = 10)
    #self.depth_pub = rospy.Publisher("/depth", Image, queue_size = 10)
    
    tss = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub],queue_size=10, slop=0.5)
    #self.callback_d(image_sub,depth_sub)                                    
    tss.registerCallback(self.callback)
    # print(self.depth)
    
    print('init')

  def callback(self, img, depth, debug=False):

    self.flag = 1

    try:
      cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
    except CvBridgeError as e:
      print(e)
    # self.image_pub.publish(cv_image)


    try:
      depth_image_raw = self.bridge.imgmsg_to_cv2(depth, "passthrough")
      depth_image = ((255 * depth_image_raw)).astype(np.uint8)
    except CvBridgeError as e:
      print(e)
    #self.depth_pub.publish(depth_image)

    #else:
        # Save OpenCV2 image as a jpeg 
        #cv2.imwrite('camera_image.jpeg', cv_image)
	#cv2.imwrite('depth_image.jpeg', depth_image)
    if debug:
      cv2.waitKey(1)
      cv2.imshow("Image window", cv_image)
      cv2.imshow("Depth window", depth_image)
    
  
    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)


def main(args): 
  ic = image_converter()

  rospy.init_node('image_converter', anonymous=True)
  
  try:

    while True:
      #wait for signal

      #sees a thing

      #gets frame from ic. They are both global variables so whenever the callback function is called
      #these variables update.
      currentRGB = cv_image
      currentDepth = depth_image


  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

#def callback(self, data) :
    


if __name__ == '__main__':
    main(sys.argv)
