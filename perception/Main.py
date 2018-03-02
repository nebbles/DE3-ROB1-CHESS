import cv2
import rospy
from mainDetect import *
from boardClass import Board
from camera_sub import image_converter

'''
0. High-level functions
'''

def makeBoard(image):
    '''
    Takes an image of an empty board and takes care of image processing and subdividing it into 64 squares
    which are then stored in one Board object that is returned.
    :param image:
    :return:
    '''
    # Process Image: convert to B/w
    image, processedImage = processFile(image)

    # Extract chessboard from image
    extractedImage = imageAnalysis(image, processedImage)

    # Chessboard Corners
    cornersImage = extractedImage.copy()

    # Canny edge detection - find key outlines
    cannyImage = cannyEdgeDetection(extractedImage)

    # Hough line detection to find rho & theta of any lines
    h, v = houghLines(cannyImage, extractedImage)

    # Find intersection points from Hough lines and filter them
    intersections = findIntersections(h, v)

    # Assign intersections to a sorted list of lists
    corners, cornerImage = assignIntersections(extractedImage, intersections)

    # Copy original image to display on
    squareImage = image.copy()

    # Get list of Square class instances
    squares = makeSquares(corners)

    # Make a Board class from all the squares to hold information
    board = Board(squares)

    ## DEBUG
    # Show the classified squares
    board.draw(squareImage)
    cv2.imshow("Classified Squares", squareImage)

    return board

def bwe(current):
    '''
    Takes care of taking the camera picture, comparing it to the previous one, updating the BWE and returning it
    :return:
    '''
    global previous

    # TODO: Get current image

    ## DEBUG
    # Getting current image
    #currentPath = "chessboard2303test/2.jpeg"

    current = cv2.imread(current, 1)

    # Copy to detect color changes --> Attention there's a weird error when you try to use the same ones
    currentCopy = current.copy()

    # Find the centre of the image differences
    centres = detectSquareChange(previous, current)

    # Now we want to check in which square the change has happened
    matches = board.whichSquares(centres)

    # Update the BWE by looking at which squares have changed
    board.updateBWE(matches, currentCopy)

    # Print second
    bwe = board.getBWE()

    # Show BWE Update
    cv2.imshow("Updating BWE", currentCopy)

    # Make current image the previous one
    previous = current

    return bwe

def printBwe(bwe):
    print("")
    print("This is the BWE matrix: ")
    print("")
    print(bwe)

'''
1. Calibrate game by taking picture of empty chessboard
'''

# Instantiate image converter
ic = image_converter()
# Initialise rospy node
rospy.init_node('image_converter', anonymous=True)

# Get image of empty board
rgbImage, depthImage = ic.image_return()

# TODO: GET EMPTY CHESSBOARD PIC
## DEBUG
# imgPathEmpty = "chessboard2303test/0.jpeg"
# Read image of empty chessboard
# img = cv2.imread(imgPathEmpty, 1)

'''
2. Generate Board class holding information about the 64 squares

When done press any key to continue
'''

# Make the board class / This contains most of the image analysis
board = makeBoard(rgbImage)

# Press key to continue
cv2.waitKey(0)

'''
3. Populate board and assign BWE matrix
'''

# TODO: Get picture of populated board at start of game

## DEBUG
# Initialising previous variable with populated chessboard
#previousPath = "chessboard2303test/1.jpeg"
#global previous
#previous = cv2.imread(previousPath, 1)

# Get image of populated board
rgbImage, depthImage = ic.image_return()

global previous
previous = rgbImage

# Assign the initial BWE Matrix to the squares
board.assignBWE()

'''
4. Find BWE matrix by analysing current images and comparing them to previous ones
'''

try:
    # Spin rospy
    rospy.spin()

    # Get new image
    currentImage, depthImage = ic.image_return()

    # Update BWE
    bwe = bwe(currentImage)

    # Print the new BWE
    printBwe(bwe)

    # Wait for user input
    cv2.waitKey(0)
except KeyboardInterrupt:
    print("Shutting down")

cv2.destroyAllWindows()