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
# Hough lines parameters
Rres = 1
Thetares = 1*np.pi/180
Threshold = 1
minLineLength = 1
maxLineGap = 100
# Detect hough lines
lines = cv2.HoughLinesP(edges,rho = 1,theta = 1*np.pi/180,threshold = 40,minLineLength = 100,maxLineGap = 50)
N = lines.shape[0]
# Draw lines on image
print(lines)
for i in range(N):
    x1 = lines[i][0][0]
    y1 = lines[i][0][1]
    x2 = lines[i][0][2]
    y2 = lines[i][0][3]
    cv2.line(extracted,(x1,y1),(x2,y2),(255,0,0),2)
# Show image
cv2.imshow('Hough lines',extracted)

# ### Contour detection
#
# # Create copy of extracted board
# squares = extracted.copy()
# # Convert to grayscale
# squaresGray = cv2.cvtColor(squares, cv2.COLOR_RGB2GRAY)
# squaresThreshold = cv2.adaptiveThreshold(squaresGray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 1)
# # Find contours
# _, contours, hierarchy = cv2.findContours(squaresThreshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# # Initialise empty numpy array
# objects = np.zeros([img.shape[0], img.shape[1],3], 'uint8')
#
# for c in contours:
#     # Area
#     area = cv2.contourArea(c)
#     # Perimenter
#     perimeter = cv2.arcLength(c, True)
#     # Filtering the chessboard edge / Error handling as some contours are so small so as to give zero division
#     try:
#         cv2.drawContours(squares, [c], -1, color, 2)
#         print("Area: {}, perimeter: {}".format(area, perimeter))
#     except:
#         pass
#
# cv2.imshow("Squares", squares)

# Wait until random button is pressed
if cv2.waitKey(0) & 0xff == 27:
     # Kill program
     cv2.destroyAllWindows()