import cv2
import numpy as np


class Line:
    def __init__(self, rho, theta):
        self.rho = rho
        self.theta = theta
        self.center = (np.cos(theta) * rho, np.sin(theta) * rho)

    def isHorizontal(self, thresholdAngle=np.pi / 4):
        return abs(np.sin(self.theta)) > np.cos(thresholdAngle)

    def isVertical(self, thresholdAngle=np.pi / 4):
        return abs(np.cos(self.theta)) > np.cos(thresholdAngle)

    # def partitionLines(lines):
    #     h = filter(lambda x: x.isHorizontal(), lines)
    #     v = filter(lambda x: x.isVertical(), lines)
    #
    #     h = [(l._center[1], l) for l in h]
    #     v = [(l._center[0], l) for l in v]
    #
    #     h.sort()
    #     v.sort()
    #
    #     h = [l[1] for l in h]
    #     v = [l[1] for l in v]
    #
    #     return (h, v)
    #
    # def intersect(self, line):
    #     ct1 = np.cos(self.theta)
    #     st1 = np.sin(self.theta)
    #     ct2 = np.cos(line.theta)
    #     st2 = np.sin(line.theta)
    #     d = ct1 * st2 - st1 * ct2
    #     if d == 0.0: raise ValueError('parallel lines: %s, %s)' % (str(self), str(line)))
    #     x = (st2 * self.rho - st1 * line._rho) / d
    #     y = (-ct2 * self.rho + ct1 * line._rho) / d
    #     return (x, y)
    #
    # def filterCloseLines(lines, horizontal=True, threshold=40):
    #     if horizontal:
    #         item = 1
    #     else:
    #         item = 0
    #
    #     i = 0
    #     ret = []
    #
    #     while i < len(lines):
    #         itmp = i
    #         while i < len(lines) and (lines[i]._center[item] - lines[itmp]._center[item] < threshold):
    #             i += 1
    #         ret.append(lines[itmp + int((i - itmp) / 2)])
    #     return ret