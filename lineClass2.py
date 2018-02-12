import cv2
import numpy as np


class Line:
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2


        self.dy = self.y2 - self.y1
        self.dx = self.x2 - self.x1

        # if self.dy == 0:
        #     self.category = 'horizontal'
        # elif self.x1 == 0 :
        #     self.category = 'vertical'
        if abs(self.dx) > abs(self.dy):
            self.category = 'horizontal'
        else:
            self.category = 'vertical'

        self.c = self.x1 * self.y2 - self.x2 * self.y1
        # self.numOfInstances = 0

    def draw(self, image, color=(0, 0, 255), thickness=2):
        cv2.line(image, (self.x1, self.y1), (self.x2, self.y2), color, thickness)

    def intersect(self, other):
        '''Where do lines self and other intersect?
        '''

        if self.x * other.b == other.a * self.y:
            raise ValueError("Lines have the same slope.")

        a = np.array ( ( (self.x, self.y), (other.a, other.b) ) )
        b = np.array ( (-self.c, -other.c) )
        x, y = np.linalg.solve(a,b)

        return x,y