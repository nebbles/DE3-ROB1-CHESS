import cv2
import numpy as np

RGBImage = cv2.imread("chessboard2303test/0.jpeg")
depthImage = cv2.imread("Depth/depth_image1.jpeg")


#get

px = depthImage[int(140),int(140)] #replace with the centres
print(px)

print(depthImage.shape)
print(RGBImage.shape)