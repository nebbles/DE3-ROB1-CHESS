import cv2
import numpy as np
from mainDetect import processFile, imageAnalysis, cannyEdgeDetection, houghLines, findIntersections

#Read Image
img = cv2.imread("chessboardPrinted.jpg", 1)

#Process Image: convert to B/w
img, processedImage = processFile(img)

#Extract chessboard from image
extractedImage = imageAnalysis(img, processedImage)

#Canny edge detection - find key outlines
cannyImage = cannyEdgeDetection(extractedImage)

#Hough line detection to find rho & theta of any lines
h,v = houghLines(cannyImage, extractedImage)

print(" ")
print ("Horizontal Hough Lines: ")
print(h)
print(" ")
print ("Vertical Hough Lines: ")
print(v)
print(" ")

#Find intersection points from Hough lines
intersections = findIntersections(h,v)
print("Number of intersection points found and filtered: " + str(len(intersections)))

for intersection in intersections:
    cv2.circle(extractedImage, intersection,radius=3,color=(255,255,255),thickness=1)

cv2.imshow("Intersections", extractedImage)
cv2.waitKey(0)
cv2.destroyAllWindows()
