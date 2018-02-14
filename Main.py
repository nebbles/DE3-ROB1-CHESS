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

#Chessboard Corners
cornersImage = extractedImage.copy()

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

#Sort by ascending y-coordinate and then by descending x-coordinate. Origin is top-left
intersections.sort(key=lambda x: (x[1],-x[0]))

print(" ")
print ("Intersections: ")
print(intersections)

for intersection in intersections:
    cv2.circle(extractedImage, intersection,radius=3,color=(255,255,255),thickness=2)
cv2.imshow("Intersections", extractedImage)


rows = []
row = 1
column = 1

#Trying to assign Intersection points to rows (1-9) and columns (1-9)
rowAssignmentThreshold = 10
for i in range(1,len(intersections)):
    if intersections[i][1] in range(intersections[i-1][1]-rowAssignmentThreshold,intersections[i-1][1]+rowAssignmentThreshold):
        rows.append((row,column,intersections[i-1]))
        column += 1
    else:
        rows.append((row,column,intersections[i-1]))
        row += 1
        column = 1
    # For last corner
    if i == len(intersections)-1:
        print("HI")
        rows.append((row,column,intersections[i]))

print(" ")
print ("Rows: ")
for row in rows:
    print(row)

cv2.circle(extractedImage,(50,50),10,255,1)



for row in rows:
    cv2.circle(cornersImage, row[2],radius=3,color=(255,255,255),thickness=2)

cv2.imshow("Corners", cornersImage)
cv2.waitKey(0)
cv2.destroyAllWindows()
