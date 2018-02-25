from mainDetect import *

# Read Image
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

# Draw the squares and classify them (draws the square color on the image)
for square in squares:
    square.draw(squareImage)
    square.classify(squareImage)

cv2.imshow("Classified Squares", squareImage)

cv2.waitKey(0)
cv2.destroyAllWindows()