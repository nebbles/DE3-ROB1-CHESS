import cv2
from mainDetect import *
from boardClass import Board

'''
1. Generate Board class holding information about the 64 squares
'''

# Read image of empty chessboard
img = cv2.imread("chessboard2303test/0.jpeg", 1)

# Process Image: convert to B/w
img, processedImage = processFile(img)

# Extract chessboard from image
extractedImage = imageAnalysis(img, processedImage)

# Chessboard Corners
cornersImage = extractedImage.copy()

# Canny edge detection - find key outlines
cannyImage = cannyEdgeDetection(extractedImage)

# Hough line detection to find rho & theta of any lines
h,v = houghLines(cannyImage, extractedImage)

# Find intersection points from Hough lines and filter them
intersections = findIntersections(h,v)

# Assign intersections to a sorted list of lists
corners, cornerImage = assignIntersections(extractedImage, intersections)

# Copy original image to display on
squareImage = img.copy()

# Get list of Square class instances
squares = makeSquares(corners)

# Make a Board class from all the squares to hold information
board = Board(squares)

## DEBUG
# Show the classified squares
#board.draw(squareImage)
#cv2.imshow("Classified Squares", squareImage)

'''
2. Start game by taking picture of populated chessboard
'''

# Assign the initial BWE Matrix to the squares
board.assignBWE()

# TODO: Get picture of populated board at start of game

# Initialising previous variable with empty chessboard
previous = cv2.imread("chessboard2303test/1.jpeg", 1)

'''
3. Find BWE matrix by analysing current images and comparing them to previous ones
'''

# This will need to be a loop at some point that does its calculations as soon as it
# receives the new image from the camera --> Sylvia's code
#while True:

# Getting current image
current = cv2.imread("chessboard2303test/2.jpeg", 1)
# Copy to detect color changes --> Attention there's a weird error when you try to use the same ones
currentCopy = current.copy()

# Find the centre of the image differences
centres = detectSquareChange(previous, current)

# Now we want to check in which square the change has happened
matches = board.whichSquares(centres)

# Print first
board.getBWE()

# Update the BWE by looking at which squares have changed
board.updateBWE(matches, currentCopy)

# Print second
board.getBWE()

# Show BWE Update
cv2.imshow("Updating BWE", currentCopy)


# Wait for user input
cv2.waitKey(0)
cv2.destroyAllWindows()