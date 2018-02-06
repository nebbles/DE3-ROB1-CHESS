import cv2
import numpy as np
import copy

from PaoloAttempt0602.board import Board
from PaoloAttempt0602.extract import extractBoards, extractGrid, extractTiles, ignoreContours, largestContour
from PaoloAttempt0602.util import showImage, drawPerspective, drawBoundaries, drawLines, drawPoint, drawContour, randomColor
from PaoloAttempt0602.line import Line

import random
import cv2
import numpy as np
import argparse

img = cv2.imread('emptyBoardCropped.jpg',1)
img_orig = copy.copy(img)

extractBoards(img, 640, 400)

cv2.waitKey(0)
cv2.destroyAllWindows()