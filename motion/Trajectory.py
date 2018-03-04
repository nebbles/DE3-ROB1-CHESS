from numpy import linspace, sqrt, concatenate
from matplotlib import interactive
from matplotlib.pyplot import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # for: fig.gca(projection = '3d')
from scipy import interpolate
from franka.franka_control import FrankaControl


def output(move, board_points, dead_zone, rest, hover, visual_flag=False):
    """ function to run the code, outputs list of vectors to complete the trajectectory"""

    # resolution - distance between each point in m
    n = 0.002

    # Use logic to extract information from move
    dead_status, start_AN, goal_AN, died_AN = logic(move)

    # Find the necessary coordinates
    start = AN_to_coords(start_AN, board_points)
    goal = AN_to_coords(goal_AN, board_points)

    # create complete array of intermediary poses
    rest_h, start_h, goal_h, deadzone_h = intermediate_coords(rest, start, goal, hover, dead_zone)

    # generate vectors and coordinates making up the path
    line_list_current = start_path(dead_status, rest, start_h, goal_h, n) # start path
    if dead_status == "Died":
        line_list_dead = dead_path(rest, goal, goal_h, dead_zone, deadzone_h, start_h, n) # dead path
        line_list_current = line_list_current + line_list_dead
    line_list_move = move_path(rest, start, start_h, goal, goal_h, n) # move path
    line_list = line_list_current + line_list_move

    # Finding the separate trajectories that surround gripping and ungripping
    if dead_status == 'None died':
        trajectory, trajectory_1, trajectory_2, trajectory_3 = generate_trajectory(line_list, dead_status)
    elif dead_status == "Died":
        trajectory, trajectory_1, trajectory_2, trajectory_3, trajectory_4, trajectory_5 = generate_trajectory(line_list, dead_status)

    # Finding the smooth trajectory
    x_sample, y_sample, z_sample = data_split(trajectory)
    x_fine, y_fine, z_fine, x_knots, y_knots, z_knots = data_interpolation(trajectory, x_sample, y_sample, z_sample)
    smooth_trajectory = generate_smooth_trajectory(x_fine, y_fine, z_fine)
    x_smooth, y_smooth, z_smooth = data_split(smooth_trajectory)

    generate_vectors(smooth_trajectory)

    # plot the trajectory
    if visual_flag:
        plot(x_sample, y_sample, z_sample, x_knots, y_knots, z_knots, x_smooth, y_smooth, z_smooth)

    return smooth_trajectory


def best_fit(X, Y):

    xbar = sum(X)/len(X)
    ybar = sum(Y)/len(Y)
    n = len(X) # or len(Y)

    numer = sum([xi*yi for xi,yi in zip(X, Y)]) - n * xbar * ybar
    denum = sum([xi**2 for xi in X]) - n * xbar**2

    b = numer / denum
    a = ybar - b * xbar

    print('best fit line:\ny = {:.2f} + {:.2f}x'.format(a, b))

    return a, b


def logic(move): # move is a list of tuple(s) [(‘R’.’a4’)(‘p’, ‘a2a4’)]
    """Function to extract information from move"""
    if len(move)==1:

        # extract ANs
        start_AN = (move[0][1])[:2]
        goal_AN = (move[0][1])[2:4]

        return "None died", start_AN, goal_AN, None # no died_AN

    elif len(move)==2:

        # extract ANs
        start_AN = (move[1][1])[:2]
        goal_AN = (move[1][1])[2:4]
        died_AN = (move[0][1])
        return "Died", start_AN, goal_AN, died_AN


def AN_to_coords(AN, board_points): #AN = a6
    """Function to convert the given algebraic notation into real world coordinates"""

    A1 = board_points[0]
    A8 = board_points[1]
    H8 = board_points[2]
    H1 = board_points[3]

    x = H8[0] - A1[0] # square width in FRANKA units
    y = A8[1] - A1[1] # square length in FRANKA units

    n = 8 # number of squares

    # find coordinates of each AN
    letters = dict([('a', x / 2), ('b', (x / 2) * 2), ('c', (x / 2) * 3), ('d', (x / 2) * 4), ('e', (x / 2) * 5),
                    ('f', (x / 2) * 6), ('g', (x / 2) * 7), ('h', (x / 2) * 8)])
    numbers = dict([('1', y / 2), ('2', (y / 2) * 2), ('3', (y / 2) * 3), ('4', (y / 2) * 4), ('5', (y / 2) * 5),
                    ('6', (y / 2) * 6), ('7', (y / 2) * 7), ('8', (y / 2) * 8)])

    # selecting the location
    letter = AN[0]
    number = AN[1]
    x = numbers[number]
    y = letters[letter]
    z = 0

    coord = [x,y,z]

    return coord

#def gripper_height():
    # Finding the gripper height depending on game engine output
    # dictionary of heights for each piece

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
    deadzone_h = [dead_zone[0], dead_zone[1], hover]

    return rest_h, start_h, goal_h, deadzone_h


def create_line(a, b, n):
    """function to create a list of coordinates between 2 points in space"""

    vector = [b[0] - a[0], b[1] - a[1], b[2] - a[2]]  # vector from b to a
    distance = sqrt(sum(i ** 2 for i in vector))

    # number of points on line
    i = int(distance/n)

    # pre-allocate space
    line = np.zeros((i, 3))

    # generate array of poses
    line_x = linspace(a[0], b[0], i)
    line_y = linspace(a[1], b[1], i)
    line_z = linspace(a[2], b[2], i)

    # append coordinates
    for j in range(i):
        line[j, 0] = line_x[j]
        line[j, 1] = line_y[j]
        line[j, 2] = line_z[j]
    print(line)
    return line


def start_path(dead_status, rest, start_h, goal_h, n):

    if dead_status == 'None died':
        l0 = create_line(rest, start_h, n)
    elif dead_status == 'Died':
        l0 = create_line(rest, goal_h, n)
    return [l0]


def move_path(rest, start, start_h, goal, goal_h, n):

    l1 = create_line(start_h, start, n)
    # grip
    l2 = create_line(start, start_h, n)
    l3 = create_line(start_h, goal_h, n)
    l4 = create_line(goal_h, goal, n)
    # ungrip
    l5 = create_line(goal, goal_h, n)
    l6 = create_line(goal_h, rest, n)

    # Remove duplicates
    line_list = [l1, l2, l3, l4, l5, l6]

    lines = []
    for line in line_list:
        line = np.delete(line, 0, 0)
        lines.append(line)

    return lines


def dead_path(rest, goal, died_h, dead_zone, dead_zone_h, start_h, n):

    l1 = create_line(rest, died_h, n)
    l2 = create_line(died_h, goal, n)
    # grip
    l3 = create_line(goal, died_h, n)
    l4 = create_line(died_h, dead_zone_h, n)
    l5 = create_line(dead_zone_h, dead_zone, n)
    # ungrip
    l6 = create_line(dead_zone, dead_zone_h, n)
    l7 = create_line(dead_zone_h, start_h, n)

    # Remove duplicates
    line_list = [l1, l2, l3, l4, l5, l6, l7]
    lines = []
    for line in line_list:
        line = np.delete(line, 0, 0)
        lines.append(line)

    return lines


def generate_trajectory(line_list, dead_status):
    """Function that generates full trajectory from individual lines"""

    trajectory = concatenate(line_list, axis=0)

    # joining the lines into a trajectory
    if dead_status == 'None died':

        # Forming the 3 movements that surround the gripping and ungripping
        line_list_1 = (line_list[0], line_list[1])
        line_list_2 = (line_list[2], line_list[3], line_list[4])
        line_list_3 = (line_list[5], line_list[6])

        trajectory_1 = concatenate(line_list_1, axis=0)
        trajectory_2 = concatenate(line_list_2, axis=0)
        trajectory_3 = concatenate(line_list_3, axis=0)

        return trajectory, trajectory_1, trajectory_2, trajectory_3

    elif dead_status == 'Died':

        # Forming the 2 movements that surround the gripping and ungripping to remove a dead piece
        line_list_1 = (line_list[0], line_list[1])
        line_list_2 = (line_list[2], line_list[3], line_list[4])
        line_list_3 = (line_list[5], line_list[6], line_list[7])
        line_list_4 = (line_list[8], line_list[9], line_list[10])
        line_list_5 = (line_list[11], line_list[12])

        trajectory_1 = concatenate(line_list_1, axis=0)
        trajectory_2 = concatenate(line_list_2, axis=0)
        trajectory_3 = concatenate(line_list_3, axis=0)
        trajectory_4 = concatenate(line_list_4, axis=0)
        trajectory_5 = concatenate(line_list_5, axis=0)

        return trajectory, trajectory_1, trajectory_2, trajectory_3, trajectory_4, trajectory_5


def data_split(trajectory):
    """function that splits the data into x, y and z's"""

    x_sample = [(trajectory[i][0]) for i in range(len(trajectory))]
    y_sample = [(trajectory[i][1]) for i in range(len(trajectory))]
    z_sample = [(trajectory[i][2]) for i in range(len(trajectory))]
    return x_sample, y_sample, z_sample


def data_interpolation(trajectory, x_sample, y_sample, z_sample):
    """function that performs interpolation to find smoothed trajectory"""

    tck, u = interpolate.splprep([x_sample, y_sample, z_sample], s=0.0005)  # s is amount of smoothness
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

    smooth_vectors = []
    for i in range(1, len(smooth_trajectory)):
        previous = smooth_trajectory[i-1]
        current = smooth_trajectory[i]

        smooth_vector = [previous[0] - current[0], previous[1] - current[1], previous[2] - current[2]]

        smooth_vectors.append(smooth_vector)

    return smooth_vectors


def plot(x_sample, y_sample, z_sample, x_knots, y_knots, z_knots, x_smooth, y_smooth, z_smooth):
    """Function that plots trajectory"""
    fig = plt.figure()
    ax3d = fig.add_subplot(111, projection='3d')
    ax3d.plot(x_sample, y_sample, z_sample, 'r')  # plotting the angled points
    #ax3d.plot(x_knots, y_knots, z_knots, 'go')  # plotting the curve knots
    ax3d.plot(x_smooth, y_smooth, z_smooth, 'g')  # plotting the smoothed trajectory
    plt.show()


if __name__ == '__main__':
    #output([("r", "b4"),("r", "a1a2")] ,visual_flag=True) # test for death
    output([("r", "a1a2")] ,visual_flag=True) # test for standard move

