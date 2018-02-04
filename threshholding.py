import cv2
import numpy as np

# read and scale down image
# wget https://bigsnarf.files.wordpress.com/2017/05/hammer.png
img = cv2.pyrDown(cv2.imread('emptyBoard.jpg', cv2.IMREAD_UNCHANGED))

# threshold image
ret, threshed_img = cv2.threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),
                                  127, 255, cv2.THRESH_BINARY)
# find contours and get the external one
image, contours, hier = cv2.findContours(threshed_img, cv2.RETR_TREE,
                                         cv2.CHAIN_APPROX_NONE)

# with each contour, draw boundingRect in green
# a minAreaRect in red and
# a minEnclosingCircle in blue

print(len(contours))
cv2.drawContours(img, contours, -1, (255, 255, 0), 1)

cv2.imshow("contours", img)

cv2.waitKey(0)
cv2.destroyAllWindows()