import cv2
import numpy as np
import random

class Square:
    def __init__(self, position, c1, c2, c3, c4):
        # ID
        self.position = position
        # Corners
        self.c1 = c1
        self.c2 = c2
        self.c3 = c3
        self.c4 = c4

        # Actual polygon as a numpy array of corners
        self.contours = np.array([c1, c2, c3, c4], dtype=np.int32)

        # Properties of the contour
        self.area = cv2.contourArea(self.contours)
        self.perimeter = cv2.arcLength(self.contours, True)
        M = cv2.moments(self.contours)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # ROI is the small circle within the square on which we will do the averaging
        self.roi = (cx, cy)

    def draw(self, image, color=(0, 0, 255), thickness=2):
        cv2.drawContours(image, [self.contours], 0, color, thickness)
        ## DEBUG
        #cv2.circle(image, self.roi, 10, (0, 0, 255), 1)

    def roiColor(self, image, radius=10):
        # Initialise mask
        maskImage = np.zeros((image.shape[0], image.shape[1]), np.uint8)
        # Draw the ROI circle on the mask
        cv2.circle(maskImage, self.roi, radius, (255, 255, 255), -1)
        # Find the average color
        average_raw = cv2.mean(image, mask=maskImage)[::-1]
        # Need int format so reassign variable
        average = (int(average_raw[1]), int(average_raw[2]), int(average_raw[3]))

        ## DEBUG
        # print(average)

        return average

    def classify(self, image):
        # Find Color of ROI
        rgb = self.roiColor(image)

        # The flag will be returned and populate the BWE matrix
        flag = ''

        # Ideal RGB
        blackEmpty = (70,25,25)
        whiteEmpty = (205,155,150)

        absDiffBE = 0
        absDiffWE = 0
        for i in range(len(rgb)):
            absDiffBE += abs(blackEmpty[i]-rgb[i])
            absDiffWE += abs(whiteEmpty[i] - rgb[i])

        if absDiffBE < absDiffWE:
            flag = 'B'
            cv2.putText(image, flag, self.roi, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        else:
            flag = 'W'
            cv2.putText(image, flag, self.roi, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

        ## DEBUG
        # print(flag)
        return flag