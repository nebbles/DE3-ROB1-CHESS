import cv2
import numpy as np


class Board:
    """
    Holds all the Square instances and the BWE matrix.
    """
    def __init__(self, squares, BWEmatrix = [], leah = 'noob coder'):
        # Squares
        self.squares = squares
        # BWE Matrix
        self.BWEmatrix = BWEmatrix
        # Noob
        self.leah = leah

    def draw(self,image):
        """
        Draws the board and classifies the squares (draws the square state on the image).
        """
        for square in self.squares:
            square.draw(image)
            square.classify(image)

    def assignBWE(self):
        """
        Assigns initial setup states to squares and initialises the BWE matrix.
        """

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
        """
        Updates the BWE by looking at the two squares that have changed and determining which one is now empty. This
        relies on calculated the distance in RGB space provided by the classify function. The one with a lower distance
        to the colour of its empty square must now be empty and its old state can be assigned to the other square that
        has changed.
        """

        # Calculates distances to empty colors of squares
        distance_one = matches[0].classify(current)
        distance_two = matches[1].classify(current)

        if distance_one < distance_two:
            # Store old state
            old = matches[0].state
            # Assign new state
            matches[0].state = 'E'
            self.BWEmatrix[matches[0].index] = matches[0].state
            # Replace state of other square with the previous one of the currently white one
            matches[1].state = old
            self.BWEmatrix[matches[1].index] = matches[1].state

        else:
            # Store old state
            old = matches[1].state
            # Assign new state
            matches[1].state = 'E'
            self.BWEmatrix[matches[1].index] = matches[1].state
            # Replace state of other square with the previous one of the currently white one
            matches[0].state = old
            self.BWEmatrix[matches[0].index] = matches[0].state

    def getBWE(self):
        """
        Converts BWE from list of strings to a rotated numpy array
        """
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
        return np.rot90(bwe,k=1)

    def whichSquares(self, points):
        """
        Returns the squares which a list of points lie within. This function is needed to filter out changes in the
        images that are compared which have nothing to do with the game, e.g. an arm.
        """
        matches = []
        for square in self.squares:
            for point in points:
                dist = cv2.pointPolygonTest(square.contours,point,False)
                if dist >= 0:
                    matches.append(square)

        return matches