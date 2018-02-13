import cv2
import numpy as np
import operator
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
print ("No. of horizontal Hough Lines: ")
print(len(h))
print(" ")
print ("No. of vertical Hough Lines: ")
print(len(v))
print(" ")

#Find intersection points from Hough lines
intersections = findIntersections(h,v)
print("Number of intersection points found and filtered: " + str(len(intersections)))

for intersection in intersections:
    cv2.circle(extractedImage, intersection,radius=3,color=(255,255,255),thickness=2)

sorted(intersections)
rows = []

#Trying to assign Intersection points to rows (1-8)
for i in range(1,len(intersections)):
    row = 1
    start = intersections[0][1] # The first y coordinate
    if intersections[0][i] == intersections[0][i-1]:
        rows.append((row,intersections[i-1]))
    else:
        rows.append((row,intersections[i-1]))
        row+=1


print(intersections)

cv2.imshow("Intersections", extractedImage)
cv2.waitKey(0)
cv2.destroyAllWindows()
