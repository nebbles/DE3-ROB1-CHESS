from numpy import array, linspace, sqrt, concatenate
from matplotlib.pyplot import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # for: fig.gca(projection = '3d')

# start and goal poses
x1, y1, z1 = [300, 300, 300]
x2, y2, z2 = [100, 300, 0]
x3, y3, z3 = [500, 300, 0]
hover = 200  # hover height

# intermediate positions
start = array([x1, y1, z1])
start_h = array([x1, y1, hover])
piece_1 = array([x2, y2, z2])
piece_1_h = array([x2, y2, hover])
piece_2 = array([x3, y3, z3])
piece_2_h = array([x3, y3, hover])

positions = []

# selecting variables
dt = 0.01  # resolution in seconds
velocity = 5  # speed in mm/s


def create_line(a, b, v):
    """function to create an array of poses between 2 points in space"""
    print 'Start, End:', (a, b)
    vector = array([a[0]-b[0], a[1]-b[1], a[2]-b[2]])  # vector from b to a
    distance = sqrt(sum(i**2 for i in vector))  # straight line distance between points

    # time to complete movement
    time = v/distance

    # number of points
    n = time/dt
    n = int(n)

    # generate array of poses
    line_x = linspace(a[0], b[0], n)
    line_y = linspace(a[1], b[1], n)
    line_z = linspace(a[2], b[2], n)
    line_list = []

    for i in range(n):
        line = (line_x[i], line_y[i], line_z[i])
        line_list.append(line)
    return line_list


# create complete array of poses
line1 = concatenate((create_line(start, start_h, velocity), create_line(start_h, piece_1_h, velocity)), axis=0)

print line1

# plotting
fig = matplotlib.pyplot.figure()
ax = fig.gca(projection='3d')
ax.plot(line1[:, 0], line1[:, 1], line1[:, 2])
matplotlib.pyplot.show()
