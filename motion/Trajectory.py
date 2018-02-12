from numpy import linspace, sqrt, concatenate
from matplotlib.pyplot import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # for: fig.gca(projection = '3d')


def intermediate_coords(rest, start, goal, hover):
    """function to generate the intermediate positions of FRANKA"""
    rest_h = [rest[0], rest[1], hover]
    start_h = [start[0], start[1], hover]
    goal_h = [goal[0], goal[1], hover]
    return rest_h, start_h, goal_h


def create_line(a, b, v, vectors, dt):
    """function to create an array of poses between 2 points in space"""
    print('Start, End:', (a, b))
    vector = [b[0]-a[0], b[1]-a[1], b[2]-a[2]]  # vector from b to a
    vectors.append(vector)
    distance = sqrt(sum(i**2 for i in vector))  # straight line distance between points
    print('vector:', vector)
    # time to complete movement
    time = v/distance

    # number of points
    n = time/dt
    n = int(n)

    # pre-allocate space
    line = np.zeros((n, 3))

    # generate array of poses
    line_x = linspace(a[0], b[0], n)
    line_y = linspace(a[1], b[1], n)
    line_z = linspace(a[2], b[2], n)

    # append coordinates
    for i in range(n):
        line[i, 0] = line_x[i]
        line[i, 1] = line_y[i]
        line[i, 2] = line_z[i]
    return line

def get_test_trajectory():
    # start and goal poses
    rest = [200, 0, 500]
    start = [400, 0, 10]
    goal = [550, 200, 10]
    hover = 200  # hover height

    # selecting variables
    dt = 0.0001  # resolution in seconds
    velocity = 5  # speed in mm/s

    # create complete array of poses
    rest_h, start_h, goal_h = intermediate_coords(rest, start, goal, hover)

    # creating the lines
    vectors = []

    l1 = create_line(rest, rest_h, velocity, vectors, dt)
    l2 = create_line(rest_h, start_h, velocity, vectors, dt)
    l3 = create_line(start_h, start, velocity, vectors, dt)
    l4 = create_line(start, start_h, velocity, vectors, dt)
    l5 = create_line(start_h, goal_h, velocity, vectors, dt)
    l6 = create_line(goal_h, goal, velocity, vectors, dt)
    l7 = create_line(goal, goal_h, velocity, vectors, dt)
    l8 = create_line(goal_h, rest_h, velocity, vectors, dt)
    l9 = create_line(rest_h, rest, velocity, vectors, dt)

    print("BEN'S Vectors: ", vectors)
    return vectors

def output():
    # start and goal poses
    rest = [200, 0, 500]
    start = [400, 0, 10]
    goal = [550, 200, 10]
    hover = 200  # hover height

    # selecting variables
    dt = 0.0001  # resolution in seconds
    velocity = 5  # speed in mm/s

    # create complete array of poses
    rest_h, start_h, goal_h = intermediate_coords(rest, start, goal, hover)

    # creating the lines
    vectors = []

    l1 = create_line(rest, rest_h, velocity, vectors, dt)
    l2 = create_line(rest_h, start_h, velocity, vectors, dt)
    l3 = create_line(start_h, start, velocity, vectors, dt)
    l4 = create_line(start, start_h, velocity, vectors, dt)
    l5 = create_line(start_h, goal_h, velocity, vectors, dt)
    l6 = create_line(goal_h, goal, velocity, vectors, dt)
    l7 = create_line(goal, goal_h, velocity, vectors, dt)
    l8 = create_line(goal_h, rest_h, velocity, vectors, dt)
    l9 = create_line(rest_h, rest, velocity, vectors, dt)

    print("BEN'S Vectors: ", vectors)

    # joining the lines into a trjectory
    line_list = (l1, l2, l3, l4, l5, l6, l7, l8, l9)
    trajectory = concatenate(line_list, axis=0)

    line_list_1 = (l1, l2, l3)
    line_list_2 = (l4, l5, l6)
    line_list_3 = (l7, l8, l9)

    trajectory_1 = concatenate(line_list_1, axis=0)
    trajectory_2 = concatenate(line_list_2, axis=0)
    trajectory_3 = concatenate(line_list_3, axis=0)

    # action list
    print(trajectory_1)
    print('grip')
    print(trajectory_2)
    print('ungrip')
    print(trajectory_3)

    # plotting
    fig = matplotlib.pyplot.figure()
    ax = fig.gca(projection='3d')
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2])
    matplotlib.pyplot.show()

    return vectors


if __name__ == '__main__':
    output()
