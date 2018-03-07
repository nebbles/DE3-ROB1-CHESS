import sys
from numpy import genfromtxt
import numpy as np

my_data = genfromtxt('cal_data.csv', delimiter=',')
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

sys.exit()

A = random.rand(num_pts, 3)
one = np.ones((num_pts, 1))
A = np.column_stack([A, one])
print("A", A)
print("\n")

T = random.rand(3, 4)
xrow = np.array([0,0,0,1])
T = np.vstack([T, xrow])
print("T", T)
print("\n")

B = np.dot(A, T)
print("B", B)
print("\n")


x = np.linalg.lstsq(A, B, rcond=None)[0]
# x = np.linalg.pinv(A)*B
# x = np.linalg.qr(A)*B
print("x", x)

