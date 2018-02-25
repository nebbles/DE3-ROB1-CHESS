import numpy
import cv2

contours = [numpy.array([[1,1],[10,50],[50,50]], dtype=numpy.int32) , numpy.array([[99,99],[99,60],[60,99]], dtype=numpy.int32)]

drawing = numpy.zeros([100, 100],numpy.uint8)
for cnt in contours:
    cv2.drawContours(drawing,[cnt],0,(255,255,255),2)

cv2.imshow('output',drawing)
cv2.waitKey(0)