import cv2
import numpy as np

class Square:
    def __init__(self, c1, c2, c3, c4):
        # Corners
        self.c1 = c1
        self.c2 = c2
        self.c3 = c3
        self.c4 = c4
        # Actual polygon as a numpy array of corners
        self.contours = np.array([c1, c2, c3, c4], dtype=np.int32)

        try:
            self.area = cv2.contourArea(self.contours)
            self.perimeter = cv2.arcLength(self.contours, True)

            M = cv2.moments(self.contours)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # ROI is the small circle within the square on which we will do the averaging
            self.roi = (cx, cy)
        ## DEBUG
        except Exception as e:

            print ("This instance with corners " + str(c1) + str(c2) + str(c3) + str(c4) + " could not be initialised properly. Error message: ")
            print(e)
            print("Moments: ")
            print(M)

    def draw(self, image, color=(0, 0, 255), thickness=1):

        for c in self.contours:
            cv2.drawContours(image, [c], -1, color, thickness)
            cv2.circle(image, self.roi, 4, (0, 0, 255), -1)


