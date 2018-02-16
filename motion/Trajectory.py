from numpy import linspace, sqrt, concatenate
from matplotlib.pyplot import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # for: fig.gca(projection = '3d')
from scipy import interpolate


def intermediate_coords(rest, start, goal, hover):
    """function to generate the intermediate positions of FRANKA"""
    rest_h = [rest[0], rest[1], hover]
    start_h = [start[0], start[1], hover]
    goal_h = [goal[0], goal[1], hover]
    return rest_h, start_h, goal_h


def create_line(a, b, v, vectors, dt):
    """function to create an array of poses between 2 points in space"""
    #print('Start, End:', (a, b))
    vector = [b[0]-a[0], b[1]-a[1], b[2]-a[2]]  # vector from b to a
    vectors.append(vector)
    distance = sqrt(sum(i**2 for i in vector))  # straight line distance between points
    #print('vector:', vector)
    # time to complete movement
    time = distance/v

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


def output(visual_flag=False):
    # start and goal poses
    rest = [200, 0, 500]
    start = [400, 0, 10]
    goal = [550, 200, 10]
    hover = 200  # hover height

    # selecting variables
    dt = 0.01  # resolution in seconds 0.001
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

    # Remove duplicates
    l1 = np.delete(l1, 0, 0)
    l2 = np.delete(l2, 0, 0)
    l3 = np.delete(l3, 0, 0)
    l4 = np.delete(l4, 0, 0)
    l5 = np.delete(l5, 0, 0)
    l6 = np.delete(l6, 0, 0)
    l7 = np.delete(l7, 0, 0)
    l8 = np.delete(l8, 0, 0)
    l9 = np.delete(l9, 0, 0)

    #print("BEN'S Vectors: ", vectors)

    # joining the lines into a trjectory
    line_list = (l1, l2, l3, l4, l5, l6, l7, l8, l9)
    trajectory = concatenate(line_list, axis=0)

    # Forming the 3 movements that surround the gripping and ungripping
    line_list_1 = (l1, l2, l3)
    line_list_2 = (l4, l5, l6)
    line_list_3 = (l7, l8, l9)

    trajectory_1 = concatenate(line_list_1, axis=0)
    trajectory_2 = concatenate(line_list_2, axis=0)
    trajectory_3 = concatenate(line_list_3, axis=0)

    # splitting the data into x, y and z's
    x_sample = [(trajectory[i][0]) for i in range(len(trajectory))]
    y_sample = [(trajectory[i][1]) for i in range(len(trajectory))]
    z_sample = [(trajectory[i][2]) for i in range(len(trajectory))]

    # interpolation to find smoothed trajectory
    tck, u = interpolate.splprep([x_sample,y_sample,z_sample], s=1000000)  # s is amount of smoothness
    x_knots, y_knots, z_knots = interpolate.splev(tck[0], tck)
    u_fine = np.linspace(0, 1, len(trajectory))
    x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)

    # extracting the smooth trajectory
    smooth_trajectory = zip(x_fine, y_fine, z_fine)
    smooth_trajectory = [list(elem) for elem in smooth_trajectory]
    smooth_trajectory = smooth_trajectory[::1000] # extracting every 1000th element

    # Calculating vectors between points on smooth trajectory
    smooth_vectors=[]

    for i in range(1,len(smooth_trajectory)):

        previous = smooth_trajectory[i-1]
        current = smooth_trajectory[i]

        smooth_vector = [previous[0] - current[0], previous[1] - current[1], previous[2] - current[2]]

        smooth_vectors.append(smooth_vector)

    if visual_flag:

        print("\nBEN'S Smooth Vectors:\n ", smooth_vectors)

        # action list
        print('\n-------------------------------------------------------------------\nSeries of movements:\n')
        print('Trajectory 1:\n', trajectory_1)
        print('grip')
        print('Trajectory 2:\n', trajectory_2)
        print('ungrip')
        print('Trajectory 3:\n', trajectory_3)

        # plotting
        fig = plt.figure()
        ax3d = fig.add_subplot(111, projection='3d')
        ax3d.plot(x_sample, y_sample, z_sample, 'r*')  # plotting the angled points
        ax3d.plot(x_knots, y_knots, z_knots, 'go')  # plotting the curve knots
        ax3d.plot(x_fine, y_fine, z_fine, 'g') # plotting the smoothed trajectory
        plt.show()

    return smooth_vectors


if __name__ == '__main__':
    output(visual_flag=True)


