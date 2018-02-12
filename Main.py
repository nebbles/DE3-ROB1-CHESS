import cv2
import numpy as np
#from lineClass import Line
from mainDetect import processFile, imageAnalysis, cannyEdgeDetection, houghLines, houghLine2

#Read Image
img = cv2.imread("chessboardPrinted.jpg", 1)

#Process Image: convert to B/w
img, processedImage = processFile(img)

#Extract chessboard from image
extractedImage = imageAnalysis(img, processedImage)

#Canny edge detection - find key outlines
cannyImage = cannyEdgeDetection(extractedImage)

#Hough line detection to find rho & theta of any lines
h = houghLines(cannyImage, extractedImage)
print(h)

# a, b, c = h.shape
#
# slope = []
#
# horizontal = []
# vertical = []
# for i in range(a):
#      slope = get_slope(h[i][0][0], h[i][0][1], h[i][0][2], h[i][0][3])
#      if slope == 'horizontal':
#           horizontal.append(list(h[i]))
#
#      else:
#           vertical.append(list(h[i]))
#
# print(horizontal)




cv2.waitKey(0)
cv2.destroyAllWindows()
