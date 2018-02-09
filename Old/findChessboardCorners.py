import numpy as np
import cv2


img = cv2.imread("chessboardPrinted.jpg")
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
cv2.imshow("Gray", gray)

# Find the chess board corners
ret, corners = cv2.findChessboardCorners(gray, (7,7),None)

# If found, add object points, image points (after refining them)
if ret == True:
    # Draw and display the corners
    img = cv2.drawChessboardCorners(img, (8,8), corners,ret)
    cv2.imshow('img',img)

cv2.waitKey(0)
cv2.destroyAllWindows()