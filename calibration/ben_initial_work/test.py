import numpy as np
from numpy import random

# MATLAB CODE
#
# A = rand(4,3);  % create a random set of 4 points
# one = ones(4,1)  % generate column of ones
# A = [A, one]  % add 1s to the set of 4 points
#
# T = rand(3,4);  % create random transformation matrix
# T = [T; 0,0,0,1]  % add 0 0 0 1 to bottom row
#
# B = A*T;  % generate target coordinates
#
# newT = A\B  % use matlab mldivide to gen transformation matrix


A = random.rand(8, 3)
one = np.ones((8, 1))
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


