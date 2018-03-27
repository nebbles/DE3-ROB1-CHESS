*****************
camera subscriber
*****************
Description
===========
In this project, a RGB-D camera was used to detect FRANKA and the chessboard. This part of the project allows us to fetch a single image from the camera livestream whenever it is needed. It is based on OpenCV and runs on ROS. RGB and depth information collected from the image frame is used in both calibration and perception.

Design
======
The external RGB-D camera is connected through USB. In order to use OpenNI-compliant devices in ROS and launch the camera drive we need to run::

 roslaunch openni2_launch openni2.launch

This part consists two classes and Queue structure was implemented. Here we use multiptocessing to spawn multiple subprocesses for parallel execution of tasks. We first initialise Queues holding RGB and depth images that need to be processed, and set the maxsize to 1. Therefore, only one image can be held at a time. ``.get()``method to retrieve the results from Queue::

    rgbFrame = self.rgb_q.get()
    depthFrame = self.depth_q.get()

The nest step is to subscribe to the camera topic in ROS. Then, in a callback function of subscribed topic, ROS images are converted into OpenCV image using CvBridge. 
..todo:: flowchart
Before we can put an item in the Queue, we need to ensure the queue is empty. If not, the item in the Queue has to be removed.
