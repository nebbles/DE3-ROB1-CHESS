from __future__ import print_function
import cv2
import time
import camera_subscriber3
from franka.franka_control_ros import FrankaRos
import rospy


def main():
    rospy.init_node('franka_python_node', anonymous=True)
    feed = camera_subscriber3.CameraFeed(debug=False)
    arm = FrankaRos()

    # Main loop - temporary
    while True:
        time.sleep(0.05)  # refresh rate of camera frames

        # rgbFrame, depthFrame = feed.get_frames()

        while feed.depth is None:
            print("depth image still none")
            time.sleep(1)

        print("X pos: ", arm.x)

        cv2.imshow("FETCHED RGB", feed.rgb)
        cv2.imshow("FETCHED Depth", feed.depth)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
