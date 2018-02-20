import cv2
import numpy as np
import operator
from mainDetect import *

#Read Image
img = cv2.imread("chessboardPrinted.jpg", 1)

#Process Image: convert to B/w
img, processedImage = processFile(img)

#Extract chessboard from image
extractedImage = imageAnalysis(img, processedImage)

#Chessboard Corners
cornersImage = extractedImage.copy()

#Canny edge detection - find key outlines
cannyImage = cannyEdgeDetection(extractedImage)

#Hough line detection to find rho & theta of any lines
h,v = houghLines(cannyImage, extractedImage)



#Find intersection points from Hough lines
intersections = findIntersections(h,v)

corners, cornerImage = assignIntersections(extractedImage, intersections)

makeSquares(corners)

##DEBUG
# print(" ")
# print ("No. of horizontal Hough Lines: ")
# print(len(h))
# print(" ")
# print ("No. of vertical Hough Lines: ")
# print(len(v))
# print(" ")
# print("Number of intersection points found and filtered: " + str(len(intersections)))

cv2.imshow("Corners", cornerImage)
cv2.waitKey(0)
cv2.destroyAllWindows()

