import cv2
from perception.mainDetect import Perception


# TODO: Sylvia: Write a function that gets an RGB image from the camera within the Perception class in mainDetect.py

"""
1. Start by getting picture of empty chessboard
"""

# TODO: Sylvia: Once you've implemented that function, get the image inside the makeBoard function and remove these lines

imgPathEmpty = "perception/chessboard2303test/0.jpeg"
depthImagePath = "perception/Depth/depth_image10.jpeg"
# Read image of empty chessboard
empty = cv2.imread(imgPathEmpty, 1)
depth = cv2.imread(depthImagePath, 1)
"""
2. Instantiate Perception object

Generates Board class within the Perception object holding information about the 64 squares
"""

# Make Perception instance
percept = Perception()

# TODO: Sylvia: makeBoard should now not take an argument anymore i.e. percept.makeBoard()

# Make a Board instance within Perception. This assigns the grid and the initial BWE given an image of an empty board
percept.makeBoard(empty, depth)

"""
3. Populate board

The board now needs to be populated in the normal setup. previous is initialised to the image with the populated
chessboard with pieces in the start positions. Current is the picture taken after a move has been made. This
needs to run in a loop so that the BWE is updated forever
"""

# TODO: Sylvia: There is something for you to do in that function

percept.initialImage()

# TODO: The following code needs to be integrated into a single function that can be called from the main script.

currentPath = "perception/chessboard2303test/2.jpeg"
# Getting current image
current = cv2.imread(currentPath, 1)

# TODO: Sylvia: You need to get percept.bwe to work without passing in current but getting the image inside the function

# Update BWE
bwe = percept.bwe(current, debug=True)

cv2.waitKey(0)
cv2.destroyAllWindows()