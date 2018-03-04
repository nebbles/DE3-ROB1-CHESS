import cv2
import numpy as np


class Board:
    '''
    Holds all the squares and the BWE matrix
    '''
    def __init__(self, squares, BWEmatrix = [], leah = 'noob coder'):
        # Squares
        self.squares = squares
        # BWE Matrix
        self.BWEmatrix = BWEmatrix
        # Noob
        self.leah = leah

    def draw(self,image):
        '''
        Draws the board and classifies the squares (draws the square state on the image)
        :param image:
        :return:
        '''
        for square in self.squares:
            square.draw(image)
            square.classify(image)

    def assignBWE(self):
        '''
        Assigns states to squares and initialises the BWE matrix
        :return:
        '''

        for i in range(8):
            self.squares[8*i + 0].state = 'W'
            self.squares[8*i + 1].state = 'W'
            self.squares[8*i + 2].state = 'E'
            self.squares[8*i + 3].state = 'E'
            self.squares[8*i + 4].state = 'E'
            self.squares[8*i + 5].state = 'E'
            self.squares[8*i + 6].state = 'B'
            self.squares[8*i + 7].state = 'B'

        for square in self.squares:
            self.BWEmatrix.append(square.state)

        return self.BWEmatrix

    def updateBWE(self, matches, current):
        for match in matches:
            state = match.classify(current, True)
            self.BWEmatrix[match.index] = state

    def getBWE(self):
        '''
        Converts BWE from list of strings to a rotated numpy array
        :return:
        '''
        bwe = np.zeros((8,8),dtype=np.int8)

        counter = 0

        for i in range(8):
            for j in range(8):
                if self.BWEmatrix[counter] == 'E':
                    tmp = 0
                elif self.BWEmatrix[counter] == 'W':
                    tmp = 1
                elif self.BWEmatrix[counter] == 'B':
                    tmp = 2
                bwe[i][j] = tmp
                counter += 1

        # Rotation in return statement
        return np.rot90(bwe)

    def whichSquares(self, points):
        '''
        Returns the squares which a list of points lie within
        :param centres:
        :return:
        '''
        matches = []
        for square in self.squares:
            for point in points:
                dist = cv2.pointPolygonTest(square.contours,point,False)
                if dist >= 0:
                    matches.append(square)

        return matches