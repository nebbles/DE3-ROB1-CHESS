import numpy as np
import cv2

img = cv2.imread('chessboardPrinted.jpg',1)
gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 1)
cv2.imshow("Binary", thresh)

_, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

img2 = img.copy()
index = -1
thickness = 4
color = (255, 0, 255)

#DRAWS ALL CONTOURS
#cv2.drawContours(img2, contours, index, color, thickness)

objects = np.zeros([img.shape[0], img.shape[1],3], 'uint8')

for c in contours:
    #AREA
    area = cv2.contourArea(c)
    #PERIMETER
    perimeter = cv2.arcLength(c, True)
    #COEFFICIENTS OF BEING LIKELY TO BE A SQUARE

    try:
        if (area / perimeter) < 50 and (area / perimeter) > 5:
            M = cv2.moments(c)
            cx = int( M['m10']/M['m00'])
            cy = int( M['m01']/M['m00'])
            #cv2.circle(objects, (cx,cy), 4, (0,0,255), -1)
            cv2.drawContours(img2, [c], -1, color, 2)
    except:
        pass

    print("Area: {}, perimeter: {}".format(area,perimeter))



cv2.imshow("Contours",img2)

cv2.waitKey(0)
cv2.destroyAllWindows()
