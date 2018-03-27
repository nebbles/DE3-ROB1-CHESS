*****************
camera subscriber
*****************
Description
===========
In this project, a RGBD camera was used to detect FRANKA and the chessboard. This part of the project allows us to fetch a single image from the camera livestream whenever it is needed. It is based on OpenCV and runs on ROS. RGB and depth information collected from the image frame is used in both calibration and perception.

Design
======
The external RGBD camera is connected through USB. In order to use OpenNI-compliant devices in ROS and launch the camera drive we need to run:
 
**run**::

 roslaunch openni2_launch openni2.launch

This part consists two classes and Queue structure was implemented. Here we use multiptocessing to spawn multiple subprocesses for parallel execution of tasks. We first initialise Queues holding RGB and depth images that need to be processed, and set the maxsize to 1. Therefore, only one image can be held at a time.

".get()" method to retrieve the results from the "Queue"
