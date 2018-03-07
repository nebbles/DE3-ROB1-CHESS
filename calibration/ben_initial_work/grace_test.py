import numpy as np
import sys

# Random data
A = np.random.rand(4, 3)
one = np.ones((4, 1))
A = np.column_stack([A, one])
print("A", A)
print("\n")

T = np.random.rand(3, 4)
xrow = np.array([0, 0, 0, 1])
T = np.vstack([T, xrow])
print("T", T)
print("\n")

B = A*T
print("B", B)
print("\n")

#### GRACE CODE INSERT HERE #######

m_list = []  # m for x, y, z
c_list = []  # c for x, y, z

for dimension in range(3):
    a = []
    b = []
    for point_index in range(len(A)):
        a.append(A[point_index][dimension])
        b.append(B[point_index][dimension])
    np.array(a)
    np.array(b)
    a = np.vstack([a, np.ones(len(a))]).T
    m, c = np.linalg.lstsq(a, b)[0]
    m_list.append(m)
    c_list.append(c)
scale = [m_list, c_list]
print("scale", scale)

input_pt = np.random.rand(1, 3)
# input_pt = np.column_stack((np.random.rand(1, 3), [1]))
print(input_pt)
output_pt = []
for di in range(3):
    output_pt.append(input_pt[di] * m_list[di] + c_list[di])

print("output", output_pt)


### TEST OUTPUTS HERE ####

p_in = np.array([255.0, 246.0, 0.0])
print(p_in.T*T)