import cv2

class Board:
    '''Holds all the squares and the BWE matrix'''
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

        print("This is the BWE matrix: ")
        print("")
        for i in range(8):
            print(self.BWEmatrix[8 * i + 0]+self.BWEmatrix[8 * i + 1]+self.BWEmatrix[8 * i + 2]+self.BWEmatrix[8 * i + 3]
                  +self.BWEmatrix[8 * i + 4]+self.BWEmatrix[8 * i + 5]+self.BWEmatrix[8 * i + 6]+
                  self.BWEmatrix[8 * i + 7])
        print("")

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