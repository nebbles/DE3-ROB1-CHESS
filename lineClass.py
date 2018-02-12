import cv2
import numpy as np


# class Line:
#     def __init__(self, rho, theta):
#         self.rho = rho
#         self.theta = theta
#         #Center coordinates of the line:
#         self.center = (np.cos(theta) * rho, np.sin(theta) * rho)
#
#     def horizontalLines(self, thresholdAngle=np.pi / 4):
#         #Re
#         return abs(np.sin(self.theta)) > np.cos(thresholdAngle)
#
# # def verticalLines(self, thresholdAngle=np.pi / 4):
# #     return abs(np.sin(self.theta)) < np.cos(thresholdAngle)
# #
#     # a, b, c = lines.shape
#     # for i in range(a):
#     #     rho = lines[i][0][0]
#     #     theta = lines[i][0][1]
#     #     a = np.cos(theta)
#     #     b = np.sin(theta)
#     #     x0, y0 = a * rho, b * rho
#     #     pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
#     #     pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
#     #     cv2.line(extracted, pt1, pt2, (0, 0, 255), 2, cv2.LINE_AA)
#     # # Show image
#     # cv2.imshow('Hough lines', extracted)
# def categoriseLines(lines):#
#
#     h = filter(lambda x: x.horizontalLines(), lines)
#     v = filter(lambda x: not x.horizontalLines(), lines)
#
#     # h = [(l.center[1], l) for l in h]
#     # v = [(l.center[0], l) for l in v]
#
#     # h.sort()
#     # v.sort()
#
#     # h = [l[1] for l in h]
#     # v = [l[1] for l in v]
#
#     return (h, v)
#
#     # def intersect(self, line):
#     #     ct1 = np.cos(self.theta)
#     #     st1 = np.sin(self.theta)
#     #     ct2 = np.cos(line.theta)
#     #     st2 = np.sin(line.theta)
#     #     d = ct1 * st2 - st1 * ct2
#     #     if d == 0.0: raise ValueError('parallel lines: %s, %s)' % (str(self), str(line)))
#     #     x = (st2 * self.rho - st1 * line._rho) / d
#     #     y = (-ct2 * self.rho + ct1 * line._rho) / d
#     #     return (x, y)
#     #
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
#         while i < len(lines) and (lines[i].center[item] - lines[itmp].center[item] < threshold):
#             i += 1
#         ret.append(lines[itmp + int((i - itmp) / 2)])
#     return ret