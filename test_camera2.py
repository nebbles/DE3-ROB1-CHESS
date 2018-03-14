from __future__ import print_function
import cv2
import time
import camera_subscriber3
from franka.franka_control_ros import FrankaRos
import rospy


def main():
    rospy.init_node('franka_python_node', anonymous=True)
    feed = camera_subscriber3.CameraFeed(debug=True)
    arm = FrankaRos()

    # Main loop - temporary
    while True:
        time.sleep(0.05)  # refresh rate of camera frames

        # rgbFrame, depthFrame = feed.get_frames()

        print("X pos: ", arm.x)

        rgb, depth = feed.get_frames()

        cv2.imshow("FETCHED RGB", rgb)
        cv2.imshow("FETCHED Depth", depth)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
