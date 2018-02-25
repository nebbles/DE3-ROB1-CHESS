import cv2
import numpy as np
import copy

img = cv2.imread('emptyBoard.jpg')
img_orig = copy.copy(img)
grayimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

corners = cv2.goodFeaturesToTrack(grayimg, 83, 0.05, 25)
corners = np.float32(corners)

for item in corners:
    x, y = item[0]
    cv2.circle(img, (x, y), 5, 255, -1)

cv2.imshow("Original", img_orig)
cv2.imshow("Top 10 corners", img)
if cv2.waitKey(0) & 0xff == 27:
    cv2.destroyAllWindows()