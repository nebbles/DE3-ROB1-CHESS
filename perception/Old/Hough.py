import cv2
import numpy as np

img = cv2.imread("chessboardPrinted.jpg",1)
img = cv2.GaussianBlur(img,(5,5),0)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


res,thresh = cv2.threshold(hsv[:,:,0], 25, 250, cv2.THRESH_BINARY_INV)

cv2.imshow("Thresh",thresh)

edges = cv2.Canny(img, 100, 300) # TUNE
cv2.imshow("Canny",edges)

Rres = 1
Thetares = 1*np.pi/180
Threshold = 1
minLineLength = 1
maxLineGap = 100
lines = cv2.HoughLinesP(edges,rho = 1,theta = 1*np.pi/180,threshold = 100,minLineLength = 100,maxLineGap = 50)
N = lines.shape[0]
for i in range(N):
    x1 = lines[i][0][0]
    y1 = lines[i][0][1]
    x2 = lines[i][0][2]
    y2 = lines[i][0][3]
    cv2.line(img,(x1,y1),(x2,y2),(255,0,0),2)

cv2.imshow('dst',img)
if cv2.waitKey(0) & 0xff == 27:
    cv2.destroyAllWindows()