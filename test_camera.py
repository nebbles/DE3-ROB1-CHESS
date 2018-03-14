#!/usr/bin/env python
# Benedict Greenberg, March 2018
# http://github.com/nebbles
#
# Make sure that you are running:
#   roscore
#   franka_controller_sub
#   roslaunch openni2_launch openni2.launch
from __future__ import print_function
import cv2
import rospy
import time
import camera_subscriber
from franka.franka_control_ros import FrankaRos


def main():
    rospy.init_node('franka_python_node', anonymous=True)
    feed = camera_subscriber.CameraFeed()
    arm = FrankaRos()

    # Main loop - temporary
    while True:
        start_clk = time.time()
        while time.time() < start_clk + 5:
            time.sleep(0.05)  # refresh rate of camera frames

            print("X pos: ", arm.x)

            rgb_frame, depth_frame = feed.get_frames()

            cv2.imshow("FETCHED RGB", rgb_frame)
            cv2.imshow("FETCHED Depth", depth_frame)
            cv2.waitKey(1)

        feed.close_subscribers()
        time.sleep(5)

        feed.start_subscribers()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
