from numpy import linspace, sqrt, concatenate
from matplotlib.pyplot import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # for: fig.gca(projection = '3d')
from scipy import interpolate


def output(move, visual_flag=False):
    """ function to run the code, outputs list of vectors to complete the trajectectory"""

    # start and goal poses
    rest = [200, 0, 500]
    start = [400, 0, 10]
    goal = [550, 200, 10]
    hover = 200  # hover height
    dead_zone = [200,300,0] # CHNAGECHNAGECHANGE

    # selecting variables
    dt = 0.1  # resolution in seconds 0.001
    velocity = 5  # speed in mm/s

    # create complete array of intermediary poses
    rest_h, start_h, goal_h, dead_h = intermediate_coords(rest, start, goal, hover, dead_zone)

    # Use logic to determine movement
    dead_status = logic(move)

    # algorithm to generate vectors making up the path
    vectors, line_list = generate_paths(dead_status, dead_zone, rest, start, start_h, goal, goal_h, dead_h, velocity, dt)
    #print("BEN'S Vectors: ", vectors)

    trajectory, trajectory_1, trajectory_2 = generate_trajectory(line_list, dead_status)
    x_sample, y_sample, z_sample = data_splot(trajectory)
    x_fine, y_fine, z_fine, x_knots, y_knots, z_knots = data_interpolation(trajectory, x_sample, y_sample, z_sample)
    smooth_trajectory = generate_smooth_trajectory(x_fine, y_fine, z_fine)
    x_smooth, y_smooth, z_smooth = generate_vectors(smooth_trajectory) # actually just splitting into x, y and z

    if visual_flag:
        print("\nBEN'S Smooth Trajectory:\n ", smooth_trajectory)

        # display trajectory
        plot(x_sample, y_sample, z_sample, x_knots, y_knots, z_knots, x_smooth, y_smooth, z_smooth)
        print_actions(trajectory_1,trajectory_2)

    return smooth_trajectory


def logic(move): # move is a list of tuple(s) [(‘R’.’a4’)(‘p’, ‘a2a4’)]
    """Function to decide what action needs to be made"""
    if len(move)==1:

        return("None died")
        # convert second thing in tuple into real life coords
        # set a relavent gripper height based on the first thing in tuple

    elif len(move)==2:

        return("Died")

        # Removing a piece
        # convert second thing in first tuple into real life coords
        # select a relavent gripper height based on first thing in first tuple

        # Moving a piece
        # convert second thing in second tuple into real life coords
        # set a relavent gripper height based on the first thing in second tuple


#def gripper():
    # Finding the gripper height depending on game engine output


def plot(x_sample, y_sample, z_sample, x_knots, y_knots, z_knots, x_smooth, y_smooth, z_smooth):
    """function that plots trajectory"""
    fig = plt.figure()
    ax3d = fig.add_subplot(111, projection='3d')
    ax3d.plot(x_sample, y_sample, z_sample, 'r')  # plotting the angled points
    ax3d.plot(x_knots, y_knots, z_knots, 'go')  # plotting the curve knots
    ax3d.plot(x_smooth, y_smooth, z_smooth, 'g')  # plotting the smoothed trajectory
    plt.show()


def print_actions(trajectory_1, trajectory_2, trajectory_3):
    """function that prints action list for Franka"""
    print('\n-------------------------------------------------------------------\nSeries of movements:\n')
    print('Trajectory 1:\n', trajectory_1)
    print('grip')
    print('Trajectory 2:\n', trajectory_2)
    print('ungrip')
    print('Trajectory 3:\n', trajectory_3)


def intermediate_coords(rest, start, goal, hover, dead_zone):
    """function to generate the intermediate positions of FRANKA"""
    rest_h = [rest[0], rest[1], hover]
    start_h = [start[0], start[1], hover]
    goal_h = [goal[0], goal[1], hover]
    dead_h = [dead_zone[0], dead_zone[1], hover]
    return rest_h, start_h, goal_h, dead_h

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


def generate_paths(dead_status, dead_zone, rest, start, start_h, goal, goal_h, dead_h, velocity, dt):
    """function that creates the 9 paths that make up the trajectory"""

    if dead_status == 'None died':
        vectors = []

        l1 = create_line(rest, start_h, velocity, vectors, dt)
        l2 = create_line(start_h, start, velocity, vectors, dt)
        #grip
        l3 = create_line(start, start_h, velocity, vectors, dt)
        l4 = create_line(start_h, goal_h, velocity, vectors, dt)
        l5 = create_line(goal_h, goal, velocity, vectors, dt)
        #ungrip
        l6 = create_line(goal, goal_h, velocity, vectors, dt)
        l7 = create_line(goal_h, rest, velocity, vectors, dt)

        # Remove duplicates
        l1 = np.delete(l1, 0, 0)
        l2 = np.delete(l2, 0, 0)
        l3 = np.delete(l3, 0, 0)
        l4 = np.delete(l4, 0, 0)
        l5 = np.delete(l5, 0, 0)
        l6 = np.delete(l6, 0, 0)
        l7 = np.delete(l7, 0, 0)

        line_list = (l1, l2, l3, l4, l5, l6, l7)

    elif dead_status == 'Died':
        vectors = []

        l1 = create_line(rest, start_h, velocity, vectors, dt)
        l2 = create_line(start_h, start, velocity, vectors, dt)
        # grip
        l3 = create_line(start, start_h, velocity, vectors, dt)
        l4 = create_line(start_h, dead_h, velocity, vectors, dt)
        l5 = create_line(dead_h, dead_zone, velocity, vectors, dt)
        # ungrip

        # Remove duplicates
        l1 = np.delete(l1, 0, 0)
        l2 = np.delete(l2, 0, 0)
        l3 = np.delete(l3, 0, 0)
        l4 = np.delete(l4, 0, 0)
        l5 = np.delete(l5, 0, 0)

        line_list = (l1, l2, l3, l4, l5)

    return vectors, line_list


def generate_trajectory(line_list, dead_status):
    """function that generate full trajectroy from different paths"""

    trajectory = concatenate(line_list, axis=0)

    # joining the lines into a trajectory
    if dead_status =='None died':

        # Forming the 3 movements that surround the gripping and ungripping
        line_list_1 = (line_list[0], line_list[1])
        line_list_2 = (line_list[2], line_list[3], line_list[4])
        line_list_3 = (line_list[5], line_list[6])

        trajectory_1 = concatenate(line_list_1, axis=0)
        trajectory_2 = concatenate(line_list_2, axis=0)
        trajectory_3 = concatenate(line_list_3, axis=0)

        return trajectory, trajectory_1, trajectory_2, trajectory_3

    elif dead_status == 'Died':

        # Forming the 2 movements that surround the gripping and ungripping
        line_list_1 = (line_list[0], line_list[1])
        line_list_2 = (line_list[2], line_list[3], line_list[4])

        trajectory_1 = concatenate(line_list_1, axis=0)
        trajectory_2 = concatenate(line_list_2, axis=0)

        return trajectory, trajectory_1, trajectory_2

def data_splot(trajectory):
    """function that splits the data into x, y and z's"""
    x_sample = [(trajectory[i][0]) for i in range(len(trajectory))]
    y_sample = [(trajectory[i][1]) for i in range(len(trajectory))]
    z_sample = [(trajectory[i][2]) for i in range(len(trajectory))]
    return x_sample, y_sample, z_sample


def data_interpolation(trajectory, x_sample, y_sample, z_sample):
    """function that performs interpolation to find smoothed trajectory"""
    tck, u = interpolate.splprep([x_sample, y_sample, z_sample], s=10000)  # s is amount of smoothness
    x_knots, y_knots, z_knots = interpolate.splev(tck[0], tck)
    u_fine = np.linspace(0, 1, len(trajectory))
    x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)
    return x_fine, y_fine, z_fine, x_knots, y_knots, z_knots


def generate_smooth_trajectory(x_fine, y_fine, z_fine):
    """function that extracts the smooth trajectory"""
    smooth_trajectory = zip(x_fine, y_fine, z_fine)
    smooth_trajectory = [list(elem) for elem in smooth_trajectory]
    # smooth_trajectory = smooth_trajectory[::1000] # extracting every 1000th element when a smaller dt is used
    return smooth_trajectory


def generate_vectors(smooth_trajectory):
    """function that calculates vectors between points on smooth trajectory"""
    #smooth_vectors = []
    for i in range(1, len(smooth_trajectory)):

        #previous = smooth_trajectory[i-1]
        #current = smooth_trajectory[i]

        #smooth_vector = [previous[0] - current[0], previous[1] - current[1], previous[2] - current[2]]

        #smooth_vectors.append(smooth_vector)

        # extracting x, y and z's from smooth_trajectory
        x_smooth = [(smooth_trajectory[i][0]) for i in range(len(smooth_trajectory))]
        y_smooth = [(smooth_trajectory[i][1]) for i in range(len(smooth_trajectory))]
        z_smooth = [(smooth_trajectory[i][2]) for i in range(len(smooth_trajectory))]
    return x_smooth, y_smooth, z_smooth


if __name__ == '__main__':
    output([("r", "a1a2"),("r", "a1a2")] ,visual_flag=True)

