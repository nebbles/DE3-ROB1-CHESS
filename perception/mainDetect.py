import cv2
import numpy as np
from skimage.measure import compare_ssim
import imutils
import operator
from perception.lineClass import Line, filterClose
from perception.squareClass import Square
from perception.boardClass import Board


class Perception:
    """
    The perception class contains a Board instance as well as functions needed to generate it and output
    a BWE matrix. The updating of the BWE is done within the Board class.
    """
    def __init__(self, board = 0, previous = 0):
        # The Board instance
        self.board = board

        # The previous image
        self.previous = previous

    """
    HIGH-LEVEL FUNCTIONS
    """
    def initialImage(self, initial):
        """
        This function sets the previous variable to the initial populated board. This function is deprecated.
        """

        self.previous = initial

    def makeBoard(self, image, depthImage):
        """
        Takes an image of an empty board and takes care of image processing and subdividing it into 64 squares
        which are then stored in one Board object that is returned. Expanding to depth calibration has not yet been
        finished.
        """
        try:
            # Process Image: convert to B/w
            image, processedImage = self.processFile(image)
        except Exception as e:
            print(e)
            print("There is a problem with the image...")
            print("")
            print("The image print is:")
            print(image)
            print("")

        # Extract chessboard from image
        extractedImage = self.imageAnalysis(image, processedImage, debug=False)

        # Chessboard Corners
        cornersImage = extractedImage.copy()

        # Canny edge detection - find key outlines
        cannyImage = self.cannyEdgeDetection(extractedImage)

        # Hough line detection to find rho & theta of any lines
        h, v = self.houghLines(cannyImage, extractedImage, debug=False)

        # Find intersection points from Hough lines and filter them
        intersections = self.findIntersections(h, v, extractedImage, debug=False)

        # Assign intersections to a sorted list of lists
        corners, cornerImage = self.assignIntersections(extractedImage, intersections, debug=False)

        # Copy original image to display on
        squareImage = image.copy()

        # Get list of Square class instances
        squares = self.makeSquares(corners, depthImage, squareImage, debug=False)

        # Make a Board class from all the squares to hold information
        self.board = Board(squares)

        # Assign the initial BWE Matrix to the squares
        self.board.assignBWE()

        print("")
        print("The initial BWE has been assigned as: ")
        self.printBwe(self.board.getBWE())
        print("")

        ## DEBUG
        # Show the classified squares
        #self.board.draw(squareImage)
        #self.board.draw(depthImage)

        #cv2.imshow("Classified Squares", squareImage)
        #cv2.imshow("Classified Squares", depthImage)

        cv2.imwrite("ClassifiedSquares.jpeg", squareImage)
        cv2.imwrite("ClassifiedDepth.jpeg", depthImage)

    def bwe(self, current, debug=False):
        """
        Takes care of taking the camera picture, comparing it to the previous one, updating the BWE and returning it.
        """

        ## DEBUG
        # Getting current image
        # currentPath = "chessboard2303test/2.jpeg"

        # Copy to detect color changes --> Attention there's a weird error when you try to use the same ones
        currentCopy = current.copy()

        # Find the centre of the image differences
        centres = self.detectSquareChange(self.previous, current)

        # Now we want to check in which square the change has happened
        matches = self.board.whichSquares(centres)

        if len(matches) > 2:
            print("")
            print("Error: More than two squares have changed!")
            print("")

        # Get the old BWE to handle errors
        old_bwe = self.board.getBWE()

        # Update the BWE by looking at which squares have changed
        self.board.updateBWE(matches, currentCopy)

        # Print second
        bwe = self.board.getBWE()

        # A change has been detected
        if not (old_bwe==bwe).all():
            # Show BWE Update
            cv2.imshow("Updating BWE", currentCopy)

            # Make current image the previous one
            self.previous = current
            success = True

        # No change has been detected
        else:
            print("WARNING: No change has been detected. The BWE has not been updated.")
            success = False

        if debug and success:
            self.printBwe(bwe)

        return bwe, success

    def printBwe(self, bwe):
        """
        Prints the BWE.
        """
        print("")
        print("BWE matrix: ")
        print("")
        print(bwe)

    """
    IMAGE PROCESSING
    """

    def processFile(self, img, debug=False):
        """
        Converts input image to grayscale & applies adaptive thresholding.
        """
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

        if debug:
            cv2.imshow("Adaptive Thresholding", adaptiveThresh)

        return img, adaptiveThresh


    def imageAnalysis(self, img, processedImage, debug=False):
        """
        Finds the contours in the chessboard, filters the largest one (the chessboard) and masks it.
        """

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
        color = (255,255,255)
        if debug:
            cv2.drawContours(imgContours, [largest], -1, color, 2)

        # Epsilon parameter needed to fit contour to polygon
        epsilon = 0.1 * Lperimeter
        # Approximates a polygon from chessboard edge
        chessboardEdge = cv2.approxPolyDP(largest, epsilon, True)


        # DEBUG
        if debug:
            cv2.drawContours(imgContours, [chessboardEdge], -1, color, 2)

        # Draw chessboard edges and assign to region of interest (ROI)
        roi = cv2.polylines(imgContours,[chessboardEdge],True,(0,255,255),thickness=3)

        # Show filtered contoured image

        #DEBUG
        if debug:
            #cv2.imshow("0 Filtered Contours", imgContours)
            cv2.imwrite("0FilteredContours.jpeg", imgContours)

        # Create new all black image
        mask = np.zeros((img.shape[0], img.shape[1]), 'uint8')*125
        # Copy the chessboard edges as a filled white polygon
        cv2.fillConvexPoly(mask, chessboardEdge, 255, 1)
        # Assign all pixels to out that are white (i.e the polygon, i.e. the chessboard)
        extracted = np.zeros_like(img)
        extracted[mask == 255] = img[mask == 255]
        # Make mask green in order to facilitate removal of the red strip around chessboard
        extracted[np.where((extracted == [125, 125, 125]).all(axis=2))] = [0, 0, 20]
        # Adds same coloured line to remove red strip based on chessboard edge
        cv2.polylines(extracted, [chessboardEdge], True, (0, 255, 0), thickness=5)

        if debug:
            #cv2.imshow("1 Masked", extracted)
            cv2.imwrite("1ExtractedMask.jpeg", extracted)

        return extracted

    def cannyEdgeDetection(self, image, debug=False):
        """
        Runs Canny edge detection
        """
        # Canny edge detection
        edges = cv2.Canny(image, 100, 300)

        ##DEBUG
        if debug:
            cv2.imshow("Canny", edges)

        return edges

    """
    HOUGH LINES
    """

    def categoriseLines(self, lines, debug=False):
        """
        Sorts the lines into horizontal & Vertical. Then sorts the lines based on their respective centers
        (x for vertical, y for horizontal).
        """
        horizontal = []
        vertical = []
        for i in range(len(lines)):
            if lines[i].category == 'horizontal':
                horizontal.append(lines[i])
            else:
                vertical.append(lines[i])

        #takes center of line & sorts
        if debug:
            print(horizontal)

        horizontal = sorted(horizontal, key=operator.attrgetter('centerH'))
        vertical = sorted(vertical, key=operator.attrgetter('centerV'))

        # sorted(horizontal, key=lambda l: l.getCenterH)
        #
        # horizontal = [(l.getCenter[1], l) for l in horizontal]
        # vertical = [(l.getCenter[0], l) for l in vertical]
        # print(horizontal)
        # # horizontal.sort()
        # # vertical.sort()
        #
        # horizontal = [l[1] for l in horizontal]
        # vertical = [l[1] for l in vertical]

        return horizontal,vertical

    def houghLines(self, edges, image, debug=True):
        """
        Detects Hough lines
        """

        # Detect hough lines
        lines = cv2.HoughLinesP(edges, rho=1, theta=1 * np.pi / 180, threshold=40, minLineLength=100, maxLineGap=50)
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
        horizontal, vertical = self.categoriseLines(lines)
        # Filter out close lines based to achieve 9

        # STANDARD THRESHOLD SHOULD BE 20
        ver = filterClose(vertical, horizontal=False, threshold=20)
        hor = filterClose(horizontal, horizontal=True, threshold=20)
        #print(len(ver))
        #print(len(hor))
        # DEBUG TO SHOW LINES
        if debug:
            debugImg = image.copy()
            self.drawLines(debugImg, ver)
            self.drawLines(debugImg, hor)
            #cv2.imshow("2 Hough Lines Found", debugImg)
            cv2.imwrite("2HoughLinesFound.jpeg", debugImg)


        return hor, ver

    def drawLines(self, image, lines, color=(0,0,255), thickness=2):
        """
        Draws lines. This function was used to debug Hough Lines generation.
        """
        #print("Going to print: ", len(lines))
        for l in lines:
            l.draw(image, color, thickness)
            ## DEBUG
            cv2.imshow('image', image)

    """
    INTERSECTIONS
    """

    def findIntersections(self, horizontals,verticals, image, debug=True):
        """
        Finds intersections between Hough lines and filters out close points. The filter relies on a computationally
        expensive for loop and could definitely be improved.
        """
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

                #print(x,y)

        if debug:
            print("")
            print("We have found: " + str(len(intersections)) + " intersections.")
            print("")

            debugImg = image.copy()

            for intersection in intersections:
                cv2.circle(debugImg, intersection, 10, 255, 1)

            #cv2.imshow("3 Intersections Found", debugImg)
            cv2.imwrite("3IntersectionsFound.jpeg", debugImg)

        ### FILTER

        # Filtering intersection points
        minDistance = 15

        # Only works if you run it several times -- WHY? Very inefficient
        # Now also works if run only once so comment the loop out
        for i in range(3):
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
        if debug:
            print("")
            print("We have filtered: " + str(len(filteredIntersections)) + " intersections.")
            print("")

        return filteredIntersections

    def assignIntersections(self, image, intersections, debug=True):
        """
        Takes the filtered intersections and assigns them to a list containing nine sorted lists, each one representing
        one row of sorted corners. The first list for instance contains the nine corners of the first row sorted
        in an ascending fashion. This function necessitates that the chessboard's horizontal lines are exactly
        horizontal on the camera image, for the purposes of row assignment.
        """

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
        if debug:

            cornerCounter = 0

            debugImg = image.copy()
            for row in corners:
                for corner in row:
                    cv2.circle(debugImg, corner, 10, 255, 1)
                    cornerCounter += 1

            #cv2.imshow("4 Final Corners", debugImg)
            cv2.imwrite("4FinalCorners.jpeg", debugImg)

            print("")
            print("There are: " + str(cornerCounter) + " corners that were found.")
            print("")


        return corners, image

    """
    SQUARE INSTANTIATION
    """

    def makeSquares(self, corners, depthImage, image, debug=True):
        """
        Instantiates the 64 squares given 81 corner points.
        """

        # List of Square objects
        squares = []
        coordinates = []
        # Lists containing positional and index information
        letters = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
        numbers = ['1', '2', '3', '4', '5', '6', '7', '8']
        index = 0

        #print(corners)

        for i in range(8):
            for j in range(8):
                # Make the square - yay!
                position = letters[-i-1] + numbers[-j-1]
                c1 = corners[i][j]
                c2 = corners[i][j+1]
                c3 = corners[i+1][j+1]
                c4 = corners[i+1][j]
                square = Square(position, c1, c2, c3, c4, index, image)
                #print(c1, c2, c3, c4)
                squares.append(square)

                square.draw(image)

                index += 1
                #print(index)
                #xyz = square.getDepth(depthImage)
                #coordinates.append(xyz)

        cv2.imshow("Board Identified", image)

        if debug:
            cv2.imwrite("5SquaresIdentified.jpeg", image)





        # Get x,y,z coordinates from square centers & depth image
        #coordinates = self.getDepth(square.roi, depthImage)
        ## DEBUG
        if debug:
            print("Number of Squares found: " + str(len(squares)))

        return squares

    def detectSquareChange(self, previous, current, debug=True):
        """
        Take a previous and a current image and returns the squares where a change happened, i.e. a figure has been
        moved from or to.
        """

        debugImg = current.copy()

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
                if area > 100:
                    (x, y, w, h) = cv2.boundingRect(c)
                    centre = (int(x + w / 2), int(y + h / 2))
                    centres.append(centre)
                    cv2.circle(debugImg, centre, 3, 255, 2)
                    cv2.rectangle(debugImg, (x, y), (x + w, y + h), (0, 0, 255), 2)
            except:
                pass

        ## DEBUG
        if debug:
            cv2.imshow("Detected Move", debugImg)

        return centres


    def showImage(self, image, name="image"):
        """
        Shows the image
        """
        #print("Showing image: '%s'" % name)
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        cv2.imshow('image', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

