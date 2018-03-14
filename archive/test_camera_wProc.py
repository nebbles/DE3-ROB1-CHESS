from __future__ import print_function
import cv2
import time
import camera_subscriber

def main():
	feed = camera_subscriber.CameraFeed()
	feed.start_process()

	# Main loop - temporary
	while True:
		time.sleep(0.05)  # refresh rate of camera frames

		rgbFrame, depthFrame = feed.get_frames()

		cv2.imshow("FETCHED RGB", rgbFrame)
		cv2.imshow("FETCHED Depth", depthFrame)
		cv2.waitKey(1)

if __name__ == '__main__':
	try:
		main()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()