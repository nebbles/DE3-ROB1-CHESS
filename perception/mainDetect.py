import cv2
import numpy as np
from lineClass import Line, filterClose
from squareClass import Square
from skimage.measure import compare_ssim
import imutils
from boardClass import Board
import operator

def processFile(img):
    '''
    Converts input image to grayscale & applies adaptive thresholding
    :param img:
    :return:
    '''
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

    ## DEBUG
    # cv2.imshow("Adaptive Thresholding", adaptiveThresh)
    return img, adaptiveThresh


def imageAnalysis(img, processedImage):
    '''
    Finds the contours in the chessboard
    :param img:
    :param processedImage:
    :return:
    '''

    ### CHESSBOARD EXTRACTION (Contours)

    # Find contours
    _, contours, hierarchy = cv2.findContours(processedImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Create copy of original image
    imgContours = img.copy()

    for c in range(len(contours)):
        # Area
        area = cv2.contourArea(contours[c])
        # Perimenter
        perimeter = cv2.arcLength(contours[c], True)
        # Filtering the chessboard edge / Error handling as some contours are so small so as to give zero division
        #For test values are 70-40, for Board values are 80 - 75 - will need to recalibrate if change
        #the largest square is always the largest ratio
        if c ==0:
            Lratio = 0
        if perimeter > 0:
            ratio = area / perimeter
            if ratio > Lratio:
                largest=contours[c]
                Lratio = ratio
                Lperimeter=perimeter
                Larea = area
        else:
            pass

    # DEBUG statements
    # cv2.drawContours(imgContours, [largest], -1, color, 2)

    # Epsilon parameter needed to fit contour to polygon
    epsilon = 0.1 * Lperimeter
    # Approximates a polygon from chessboard edge
    chessboardEdge = cv2.approxPolyDP(largest, epsilon, True)
    # DEBUG
    # cv2.drawContours(imgContours, [chessboardEdge], -1, color, 2)

    # Draw chessboard edges and assign to region of interest (ROI)
    roi = cv2.polylines(imgContours,[chessboardEdge],True,(0,255,255),thickness=3)

    # Show filtered contoured image

    #DEBUG
    #cv2.imshow("Filtered Contours", imgContours)
    ## DEBUG
    #cv2.imshow("Filtered Contours", imgContours)

    # Create new all black image
    mask = np.zeros((img.shape[0], img.shape[1]), 'uint8')
    # Copy the chessboard edges as a filled white polygon
    cv2.fillConvexPoly(mask, chessboardEdge, 255, 1)
    # Assign all pixels to out that are white (i.e the polygon, i.e. the chessboard)
    extracted = np.zeros_like(img)
    extracted[mask == 255] = img[mask == 255]
    # Make mask green in order to facilitate removal of the red strip around chessboard
    extracted[np.where((extracted == [0, 0, 0]).all(axis=2))] = [0, 0, 100]
    # Adds same coloured line to remove red strip based on chessboard edge
    cv2.polylines(extracted, [chessboardEdge], True, (0, 0, 100), thickness=15)

    ## DEBUG
    #cv2.imshow("Masked", extracted)

    return extracted

def cannyEdgeDetection(image):
    '''
    Runs Canny edge detection
    :param image:
    :return:
    '''
    # Canny edge detection
    edges = cv2.Canny(image, 100, 300)

    ##DEBUG
    # Show image
    #cv2.imshow("Canny", edges)
    return edges

def categoriseLines(lines):
    '''
    Sorts the lines into horizontal & Vertical. Then sorts the lines based on their respective centers
    (x for vertical, y for horizontal). H
    :param lines: detected lines on image, in lines class
    :return: horizontal lines, vertical lines
    '''
    horizontal = []
    vertical = []
    for i in range(len(lines)):
        if lines[i].category == 'horizontal':
            horizontal.append(lines[i])
        else:
            vertical.append(lines[i])

    #takes center of line & sorts
    print(horizontal)

    horizontal = sorted(horizontal, key=operator.attrgetter('centerH'))
    vertical = sorted(vertical, key=operator.attrgetter('centerV'))

    # sorted(horizontal, key=lambda l: l.getCenterH)
    #
    # horizontal = [(l.getCenter[1], l) for l in horizontal]
    # vertical = [(l.getCenter[0], l) for l in vertical]
    # #todo change to get center function;
    # print(horizontal)
    # # horizontal.sort()
    # # vertical.sort()
    #
    # horizontal = [l[1] for l in horizontal]
    # vertical = [l[1] for l in vertical]

    return horizontal,vertical

def houghLines(edges, image):
    '''
    Detects Hough lines
    :param edges:
    :param image:
    :return:
    '''

    # Detect hough lines
    lines = cv2.HoughLinesP(edges, rho=1, theta=1 * np.pi / 180, threshold=80, minLineLength=100, maxLineGap=50)
    N = lines.shape[0]

    # Draw lines on image
    New = []
    for i in range(N):
        x1 = lines[i][0][0]
        y1 = lines[i][0][1]
        x2 = lines[i][0][2]
        y2 = lines[i][0][3]

        New.append([x1,y1,x2,y2])

    lines = [Line(x1=New[i][0],y1= New[i][1], x2= New[i][2], y2=New[i][3]) for i in range(len(New))]

    # Categorise the lines into horizontal or vertical
    horizontal, vertical = categoriseLines(lines)
    # Filter out close lines based to achieve 9

    # STANDARD THRESHOLD SHOULD BE 20
    ver = filterClose(vertical, horizontal=False, threshold=20)
    hor = filterClose(horizontal, horizontal=True, threshold=20)
    #print(len(ver))
    #print(len(hor))
    # DEBUG TO SHOW LINES
    #drawLines(image, ver)
    #drawLines(image, hor)
    return hor, ver

# def showImage(image, lines):
#     cv2.line(image, (Line.p1), (Line.p2), (255, 0, 0), 2, cv2.LINE_AA)
#     cv2.imshow('Hough lines', image)

def drawLines(image, lines, color=(0,0,255), thickness=2):
    #print("Going to print: ", len(lines))
    for l in lines:
        l.draw(image, color, thickness)
        ## DEBUG
        cv2.imshow('image', image)


def findIntersections(horizontals,verticals):
    '''
    WARNING: This function is trashy af. IDK why it works but it does. Finds intersections between Hough lines
    :param lines:
    :return:
    '''
    intersections = []

    # Finding the intersection points
    for horizontal in horizontals:
        for vertical in verticals:

            d = horizontal.dy*vertical.dx-horizontal.dx*vertical.dy
            dx = horizontal.c*vertical.dx-horizontal.dx*vertical.c
            dy=horizontal.dy*vertical.c-horizontal.c*vertical.dy

            if d != 0:
                x =abs(int(dx/d))
                y= abs(int(dy/d))
            else:
                return False

            intersections.append((x,y))

    ### FILTER

    # Filtering intersection points
    minDistance = 15

    # Only works if you run it several times -- WHY? Very inefficient
    # Now also works if run only once so comment the loop out
    #for i in range(3):
    for intersection in intersections:
        for neighbor in intersections:
            distanceToNeighbour = np.sqrt((intersection[0] - neighbor[0]) ** 2 + (intersection[1] - neighbor[1]) ** 2)
            # Check that it's not comparing the same ones
            if distanceToNeighbour < minDistance and intersection != neighbor:
                intersections.remove(neighbor)

    # We still have duplicates for some reason. We'll now remove these
    filteredIntersections = []
    # Duplicate removal
    seen = set()
    for intersection in intersections:
        # If value has not been encountered yet,
        # ... add it to both list and set.
        if intersection not in seen:
            filteredIntersections.append(intersection)
            seen.add(intersection)

    ## DEBUG
    #print(len(filteredIntersections))

    return filteredIntersections

def assignIntersections(image, intersections):
    '''
    Takes the filtered intersections and assigns them to a list containing nine sorted lists, each one representing
    one row of sorted corners. The first list for instance contains the nine corners of the first row sorted
    in an ascending fashion.
    :param filteredIntersections:
    :return:
    '''

    # Corners array / Each list in list represents a row of corners
    corners = [[],[],[],[],[],[],[],[],[]]

    # Sort rows (ascending)
    intersections.sort(key=lambda x: x[1])

    # Assign rows first, afterwards it's possible to swap them around within their rows for correct sequence
    row = 0
    rowAssignmentThreshold = 10

    for i in range(1, len(intersections)):
        if intersections[i][1] in range(intersections[i - 1][1] - rowAssignmentThreshold,
                                        intersections[i - 1][1] + rowAssignmentThreshold):
            corners[row].append(intersections[i - 1])
        else:
            corners[row].append(intersections[i - 1])
            row += 1
        # For last corner
        if i == len(intersections) - 1:
            corners[row].append(intersections[i])

    # Sort by x-coordinate within row to get correct sequence
    for row in corners:
        row.sort(key=lambda x: x[0])

    ## DEBUG
    # print(corners)

    return corners, image

def makeSquares(corners):
    '''
    Instantiates the 64 squares when given 81 corner points
    :param corners:
    :return:
    '''

    # List of Square objects
    squares = []

    # Lists containing positional and index information
    letters = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
    numbers = ['1', '2', '3', '4', '5', '6', '7', '8']
    index = 0

    for i in range(8):
        for j in range(8):
            # Make the square - yay!
            position = letters[-i-1] + numbers[-j-1]
            c1 = corners[i][j]
            c2 = corners[i][j+1]
            c3 = corners[i+1][j+1]
            c4 = corners[i+1][j]
            square = Square(position, c1, c2, c3, c4, index)
            squares.append(square)
            index += 1

    ## DEBUG
    # print("Number of Squares found: " + str(len(squares)))

    return squares

def detectSquareChange(previous, current):
    '''
    Take a previous and a current image and returns the squares where a change happened, i.e. a figure has been
    moved from or to.
    :param previous:
    :param current:
    :return:
    '''

    # Convert the images to grayscale
    grayA = cv2.cvtColor(previous, cv2.COLOR_BGR2GRAY)
    grayB = cv2.cvtColor(current, cv2.COLOR_BGR2GRAY)

    # Computes the Structural Similarity Index (SSIM) between previous and current
    (score, diff) = compare_ssim(grayA, grayB, full=True)
    diff = (diff * 255).astype("uint8")

    ## DEBUG
    # print("SSIM: {}".format(score))

    # Threshold the difference image, followed by finding contours to obtain the regions of the two input images that differ
    thresh = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]

    # Loop over the contours to return centres of differences
    centres = []

    for c in cnts:
        # Compute the bounding box and find its centre
        try:
            # Area
            area = cv2.contourArea(c)
            # Perimenter
            perimeter = cv2.arcLength(c, True)
            if area > 100:
                (x, y, w, h) = cv2.boundingRect(c)
                centre = (int(x + w / 2), int(y + h / 2))
                centres.append(centre)
                cv2.circle(current, centre, 3, 255, 2)
                cv2.rectangle(current, (x, y), (x + w, y + h), (0, 0, 255), 2)
        except:
            pass

    ## DEBUG
    #cv2.imshow("Detected Move", current)

    return centres

def showImage(image, name="image"):
    #print("Showing image: '%s'" % name)
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.imshow('image', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

