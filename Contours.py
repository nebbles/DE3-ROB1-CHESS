import cv2
import numpy as np


'''
IMAGE INPUT
'''
img = cv2.imread("chessboardPrinted.jpg",1)
# Blur
img = cv2.GaussianBlur(img,(5,5),0)
# Convert to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# Convert to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
# HSV Thresholding
res,hsvThresh = cv2.threshold(hsv[:,:,0], 25, 250, cv2.THRESH_BINARY_INV)
# Show adaptively thresholded image
adaptiveThresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 1)
# Show both thresholded images
# cv2.imshow("HSV Thresholded",hsvThresh)
cv2.imshow("Adaptive Thresholding", adaptiveThresh)

'''
IMAGE ANALYSIS
'''

### CHESSBOARD EXTRACTION (Contours)

# Find contours
_, contours, hierarchy = cv2.findContours(adaptiveThresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# Create copy of original image
imgContours = img.copy()
# Contour detection parameters
index = -1
thickness = 4
color = (255, 0, 255)
# Initialise empty numpy array
objects = np.zeros([img.shape[0], img.shape[1],3], 'uint8')

for c in contours:
    # Area
    area = cv2.contourArea(c)
    # Perimenter
    perimeter = cv2.arcLength(c, True)
    # Filtering the chessboard edge / Error handling as some contours are so small so as to give zero division
    try:
        if (area / perimeter) < 70 and (area / perimeter) > 40:
            # DEBUG statements
            #cv2.drawContours(imgContours, [c], -1, color, 2)
            #print("Area: {}, perimeter: {}".format(area, perimeter))

            # Epsilon parameter needed to fit contour to polygon
            epsilon = 0.1 * perimeter
            # Approximates a polygon from chessboard edge
            chessboardEdge = cv2.approxPolyDP(c, epsilon, True)
            #DEBUG
            #cv2.drawContours(imgContours, [chessboardEdge], -1, color, 2)

            #Draw chessboard edges and assign to region of interest (ROI)
            #roi = cv2.polylines(imgContours,[chessboardEdge],True,(0,255,255),thickness=3)
    except:
        pass

# Show filtered contoured image
cv2.imshow("Filtered Contours", imgContours)

# Create new all black image
mask = np.zeros((img.shape[0], img.shape[1]), 'uint8')
# Copy the chessboard edges as a filled white polygon
cv2.fillConvexPoly(mask, chessboardEdge, 255, 1)
# Assign all pixels to out that are white (i.e the polygon, i.e. the chessboard)
extracted = np.zeros_like(img)
extracted[mask == 255] = img[mask == 255]
# Make mask green in order to facilitate removal of the red strip around chessboard
extracted[np.where((extracted==[0,0,0]).all(axis=2))] = [0,100,0]
# Adds same coloured line to remove red strip based on chessboard edge
cv2.polylines(extracted,[chessboardEdge],True,(0,100,0),thickness=6)
# Show image
cv2.imshow("Masked", extracted)

### HOUGH LINES

# Canny edge detection
edges = cv2.Canny(extracted, 100, 300)
# Show image
cv2.imshow("Canny",edges)

# kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (2, 2))
# dilated = cv2.dilate(edges, kernel, iterations=5)
# print(dilated)
# Detect hough lines
#Tuned hough line params minlineleng = 180, max= 90


lines = cv2.HoughLinesP(edges,rho = 1,theta = 1*np.pi/180,threshold = 40,
                        minLineLength = 180,maxLineGap = 90)

N = lines.shape[0]
# Draw lines on image

hough_threshold_min = int(50.0)
hough_threshold_max = int(150.0)
hough_threshold_step = int(20.0)
for i in range(5):
        lines = cv2.HoughLines(edges, 1, np.pi / 180, hough_threshold_max - (hough_threshold_step * i))
        if lines is None:
            continue

a,b,c = lines.shape
for i in range(a):
    rho = lines[i][0][0]
    theta = lines[i][0][1]
    a = np.cos(theta)
    b = np.sin(theta)
    x0, y0 = a*rho, b*rho
    pt1 = ( int(x0+1000*(-b)), int(y0+1000*(a)) )
    pt2 = ( int(x0-1000*(-b)), int(y0-1000*(a)) )
    cv2.line(extracted, pt1, pt2, (0, 0, 255), 2, cv2.LINE_AA)
# Show image
cv2.imshow('Hough lines',extracted)

# def get_slope(x1,y1,x2, y2):
#     a = float(x1) ; b = float(y1); c = float(x2) ; d=float(y2)
#     y = y2 - y1
#     x = x2 - x1
#     if y == 0 or x == 0:
#         slope = 0
#
#     else:
#         slope = y/x
#
#     return slope
#
# right_lane = []
# left_lane = []
#
# for l in lines:
#     for lines in l:
#         slope = get_slope(lines[0], lines[1], lines[2], lines[3])
#         # These lines would not be on either lanes.
#         if -0.2 < slope < 0.2:
#             continue
#




# Wait until random button is pressed
if cv2.waitKey(0) & 0xff == 27:
     # Kill program
     cv2.destroyAllWindows()