import numpy as np
import cv2


img = cv2.imread("emptyBoard.jpg",1)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

# Find the chess board corners
ret, corners = cv2.findChessboardCorners(gray, (7,7),(8,8))

# If found, add object points, image points (after refining them)
if ret == True:
    corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    # Draw and display the corners
    img = cv2.drawChessboardCorners(img, (8,8), corners2,ret)
    cv2.imshow('img',img)
    cv2.waitKey(0)

cv2.destroyAllWindows()