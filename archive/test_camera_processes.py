from __future__ import print_function
import cv2
import time
import camera_subscriber_processes as csp


def main():
    feed = csp.CameraFeed()
    feed.start_process()

    # Main loop - temporary
    while True:
        time.sleep(0.05)  # refresh rate of camera frames

        rgb_frame, depth_frame = feed.get_frames()

        cv2.imshow("FETCHED RGB", rgb_frame)
        cv2.imshow("FETCHED Depth", depth_frame)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
