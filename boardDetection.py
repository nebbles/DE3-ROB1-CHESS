import cv2
import numpy as np
import copy


img = cv2.imread('emptyBoardCropped.jpg',1)
img_orig = copy.copy(img)

#Convert to gray space
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

## Adaptive Thresholding
thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 1)
cv2.imshow("Binary", thresh)

## Create SIFT and detect key points
sift_descriptor = cv2.xfeatures2d.SIFT_create()
(kps, descs) = sift_descriptor.detectAndCompute(thresh, None)
thresh = cv2.drawKeypoints(thresh, kps, thresh, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv2.imshow('Sift Keypoints',thresh)

###HARRIS
# must give a float32 data type input
gray = np.float32(gray)
dst = cv2.cornerHarris(gray, 2, 3, 0.05)
# result is dilated for marking the corners, not important
dst = cv2.dilate(dst, None)
# Threshold for an optimal value, it may vary depending on the image.
img[dst > 0.001 * dst.max()] = [0, 0, 255]

cv2.imshow('Harris Corner Detector', img)



cv2.waitKey(0)
cv2.destroyAllWindows()