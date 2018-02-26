import cv2
import numpy as np

class Board:
    '''Holds all the squares and the BWE matrix'''
    def __init__(self, squares, BWEmatrix = ''):
        # Squares
        self.squares = squares
        # BWE Matrix
        self.BWEmatrix = BWEmatrix

    def draw(self,image):
        '''
        Draws the board and classifies the squares (draws the square state on the image)
        :param image:
        :return:
        '''
        for square in self.squares:
            square.draw(image)
            square.classify(image)

    def initBWE(self):
        '''
        Initialises the BWE matrix
        :return:
        '''
        self.BWE = ['B', 'B', 'E', 'E', 'E', 'E', 'W', 'W',
                    'B', 'B', 'E', 'E', 'E', 'E', 'W', 'W',
                    'B', 'B', 'E', 'E', 'E', 'E', 'W', 'W',
                    'B', 'B', 'E', 'E', 'E', 'E', 'W', 'W',
                    'B', 'B', 'E', 'E', 'E', 'E', 'W', 'W',
                    'B', 'B', 'E', 'E', 'E', 'E', 'W', 'W',
                    'B', 'B', 'E', 'E', 'E', 'E', 'W', 'W',
                    'B', 'B', 'E', 'E', 'E', 'E', 'W', 'W', ]

    def updateBWE(self, match):
        dummy = 1

    def whichSquares(self, points):
        '''
        Returns the squares which a list of points lie within
        :param centres:
        :return:
        '''
        match = []
        for square in self.squares:
            for point in points:
                dist = cv2.pointPolygonTest(square.contours,point,False)
                if dist >= 0:
                    match.append(square)

        return match