import cv2
import numpy as np
from lineClass import Line
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
Lines = houghLine2(cannyImage, extractedImage)

if cv2.waitKey(0) & 0xff == 27:
     # Kill program
     cv2.destroyAllWindows()