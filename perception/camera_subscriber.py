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
    # Initialise Queues holding rgb and depth image
    self.rgb_q = mp.Queue(maxsize=1)
    self.depth_q = mp.Queue(maxsize=1)

  def start_process(self):
    # Create process object
    self.camera_feed = mp.Process(target=main, args=(self.rgb_q, self.depth_q, self.debug))
    # Set the daemon to True - causes process to shutdown if parent process shuts  
    self.camera_feed.daemon = True
    # Start the process
    self.camera_feed.start()
    # Sleep to allow process to start
    time.sleep(1)

  def get_frames(self):
    while self.rgb_q.empty() and self.depth_q.empty():
      time.sleep(0.1)
    
    # we fetch latest image from queue
    rgbFrame = self.rgb_q.get()
    depthFrame = self.depth_q.get()

    return rgbFrame, depthFrame

                                                                                                             
class ImageConverter:
  def __init__(self, rgb_q, depth_q, debug=False):
    self.debug = debug
    self.rgb_q = rgb_q
    self.depth_q = depth_q
    self.bridge = CvBridge()

    image_sub = Subscriber("/camera/rgb/image_rect_color", Image)
    depth_sub = Subscriber("/camera/depth_registered/image_raw", Image)
    
    tss = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub],queue_size=10, slop=0.5)                                   
    tss.registerCallback(self.callback)
    
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

    if self.debug:
      print("Image has been converted.")

    if not self.rgb_q.empty() and not self.depth_q.empty():
      if self.debug:
        print("Queue has item in it.")
      try:
        if self.debug:
          print("Attempting to remove item from queue.")
        self.rgb_q.get(False)
        self.depth_q.get(False)
        if self.debug:
          print("Successfully removed images from queue.")
      except:
        if self.debug:
          print("\033[91m"+"Exception Empty was raised. Could not remove from queue."+"\033[0m")

    if self.debug:
      print("Queue should be empty now, putting in images.")

    self.rgb_q.put(cv_image)
    self.depth_q.put(depth_image)

    if self.debug:
      print("Images are now in queue.")

def main(rgb_q, depth_q, debug): 
  ic = ImageConverter(rgb_q, depth_q, debug=debug)
  rospy.init_node('image_converter', anonymous=True)
   
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  finally:
    cv2.destroyAllWindows()
