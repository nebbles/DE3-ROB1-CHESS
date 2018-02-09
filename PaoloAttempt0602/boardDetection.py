import cv2
import numpy as np
import copy

from PaoloAttempt0602.board import Board
from PaoloAttempt0602.extract import extractBoards, extractGrid, extractTiles, ignoreContours, largestContour
from PaoloAttempt0602.util import showImage, drawPerspective, drawBoundaries, drawLines, drawPoint, drawContour, randomColor
from PaoloAttempt0602.line import Line

import cv2

img = cv2.imread('chessboardPrinted.jpg',1)

hello = extractBoards(img, 400, 400)
print (hello)


cv2.waitKey(0)
cv2.destroyAllWindows()