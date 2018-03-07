import sys
from numpy import genfromtxt
import numpy as np

my_data = genfromtxt('cal_data_example.csv', delimiter=',')
# print(my_data)

num_pts = 4

A = my_data[0:num_pts, 0:3]
B = my_data[0:num_pts, 3:]

one = np.ones((num_pts, 1))
A = np.column_stack([A, one])
B = np.column_stack([B, one])


# print(A)
# print(B)

x = np.linalg.lstsq(A, B, rcond=None)[0]
# x = np.linalg.pinv(A)*B
# x = np.linalg.qr(A)*B
print("x", x)


sample_in = my_data[0, 0:3]
sample_in = np.append(sample_in, 1)

sample_out = np.dot(sample_in, x)
actual_out = my_data[0, 3:]
print("Pass in coordinates: ", sample_in)
print("Convert coordinates: ", sample_out)
print("Correct coordinates: ", actual_out)
