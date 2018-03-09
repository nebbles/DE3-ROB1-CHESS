from numpy import linspace, sqrt, concatenate, exp, ndarray
from matplotlib import interactive
from matplotlib.pyplot import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # for: fig.gca(projection = '3d')
from scipy import interpolate
#from franka.franka_control import FrankaControl
import decimal

def continuous_trajectory(move, board_points, dead_zone, rest, hover, visual_flag=False):
    """ Calls the other functions, outputs list of coordinates to be followed by FRANKA

    Attributes:
        * ``move``: List of tuples outputted by the game engine
        * ``board_points``, dead_zone, rest, hover: Key locations collected from FRANKA

    Returns:
        * A list of ``x,y,z`` coordinated
    """

    # resolution - distance between each point in m
    n = 0.002

    # Use logic to extract information from move
    dead_status, start_AN, goal_AN = logic(move)

    # Find the necessary coordinates
    start = AN_to_coords(start_AN, board_points)
    goal = AN_to_coords(goal_AN, board_points)

    # create complete array of intermediary poses
    rest_h, start_h, goal_h, deadzone_h = intermediate_coords(rest, start, goal, hover, dead_zone)

    # generate vectors and coordinates making up the path
    line_list_current = start_path(dead_status, rest, start_h, goal_h, n) # start path
    if dead_status == "Died":
        line_list_dead = dead_path(goal, goal_h, dead_zone, deadzone_h, start_h, n) # dead path
        line_list_current = line_list_current + line_list_dead
    line_list_move = move_path(rest, start, start_h, goal, goal_h, n) # move path
    line_list = line_list_current + line_list_move

    # Finding the separate trajectories that surround gripping and ungripping
    if dead_status == 'None died':
        trajectory, trajectory_1, trajectory_2, trajectory_3 = generate_trajectory(line_list, dead_status)
    elif dead_status == "Died":
        trajectory, trajectory_1, trajectory_2, trajectory_3, trajectory_4, trajectory_5 = generate_trajectory(line_list, dead_status)

    # Finding the smooth trajectory
    smooth_trajectory, x_sample, y_sample, z_sample, x_knots, y_knots, z_knots, x_smooth, y_smooth, z_smooth = smooth(trajectory)

    max_speed = 0.05  # m/s
    speeds = generate_speeds(max_speed, smooth_trajectory)

    # Plot the trajectory
    if visual_flag:
        fig = plt.figure()

        ax3d = fig.add_subplot(121, projection='3d')

        ax3d.plot(x_sample, y_sample, z_sample, 'r')  # plotting the angled points
        # ax3d.plot(x_knots, y_knots, z_knots, 'go')  # plotting the curve knots
        ax3d.plot(x_smooth, y_smooth, z_smooth, 'g')  # plotting the smoothed trajectory

        x_axis = [i for i in range(len(smooth_trajectory))]
        ax = fig.add_subplot(122)
        ax.plot(x_axis, speeds)

        plt.show()

    return smooth_trajectory
    #return x_smooth, y_smooth, z_smooth, speeds


def trajectory_and_gripping(move, board_points, dead_zone, rest, hover, visual_flag=False):
    """ Calls the other functions, outputs list of coordinates to be followed by FRANKA

    Attributes:
        * ``move``: List of tuples outputted by the game engine
        * ``board_points``, dead_zone, rest, hover: Key locations collected from FRANKA

    Returns:
        * A list of ``x,y,z`` coordinated
    """

    # resolution - distance between each point in m
    n = 0.002

    # Use logic to extract information from move
    dead_status, start_AN, goal_AN = logic(move)

    # Find the necessary coordinates
    start = AN_to_coords(start_AN, board_points)
    goal = AN_to_coords(goal_AN, board_points)

    # create complete array of intermediary poses
    rest_h, start_h, goal_h, deadzone_h = intermediate_coords(rest, start, goal, hover, dead_zone)

    # generate vectors and coordinates making up the path
    line_list_current = start_path(dead_status, rest, start_h, goal_h, n) # start path
    if dead_status == "Died":
        line_list_dead = dead_path(goal, goal_h, dead_zone, deadzone_h, start_h, n) # dead path
        line_list_current = line_list_current + line_list_dead
    line_list_move = move_path(rest, start, start_h, goal, goal_h, n) # move path
    line_list = line_list_current + line_list_move

    # Finding the separate trajectories that surround gripping and ungripping
    if dead_status == 'None died':
        trajectory, trajectory_1, trajectory_2, trajectory_3 = generate_trajectory(line_list, dead_status)
        actions = [trajectory_1, 'grip', trajectory_2, 'ungrip', trajectory_3]
    elif dead_status == "Died":
        trajectory, trajectory_1, trajectory_2, trajectory_3, trajectory_4, trajectory_5 = generate_trajectory(line_list, dead_status)
        actions = [trajectory_1, 'grip', trajectory_2, 'ungrip', trajectory_3, 'grip', trajectory_4, 'ungrip', trajectory_5]

    #max_speed = 0.05
    #speeds_list = []

    # Finding the smooth  trajectory
    smooth_actions = []
    i = 0
    for i in range(len(actions)):
        action = actions[i]
        if isinstance(action, ndarray): # Appending the coordinates
            smooth_trajectory, x_sample, y_sample, z_sample, x_knots, y_knots, z_knots, x_smooth, y_smooth, z_smooth = smooth(action)
            action = list(zip(x_smooth, y_smooth, z_smooth))
            #speeds = generate_speeds(max_speed, action)
            #speeds_list.append(speeds)
            smooth_actions.append(action)
        elif isinstance(action, str): # Appending the gripping command
            smooth_actions.append(action)
        i = i+1

    print(len(smooth_actions))
    print(len(actions))
    print(smooth_actions)

    # Plot the trajectory
    if visual_flag:
        fig = plt.figure()

        ax3d = fig.add_subplot(121, projection='3d')

        for i in range(len(smooth_actions)):
            action = smooth_actions[i]
            if isinstance(action, list):
                x, y, z = data_split(action)
                print(x)
                ax3d.plot(x, y, z, 'r')  # plotting the angled points
            else:
                pass
        # ax3d.plot(x_knots, y_knots, z_knots, 'go')  # plotting the curve knots
        #ax3d.plot(x_smooth, y_smooth, z_smooth, 'g')  # plotting the smoothed trajectory

        # x_axis = [i for i in range(len(smooth_trajectory))]
        # ax = fig.add_subplot(122)
        # ax.plot(x_axis, speed)

        plt.show()

    return smooth_actions
    #return x_fine, y_fine, z_fine, speeds


def logic(move):
    """Extracts information from move given by game engine

    Returns:
        * A string ``None died`` or ``Died`` accordingly
        * Algebraic notation (AN) of the start and goal
    """

    if len(move) == 1:

        # extract ANs
        start_AN = (move[0][1])[:2]
        goal_AN = (move[0][1])[2:4]

        return "None died", start_AN, goal_AN

    elif len(move) == 2:

        # extract ANs
        start_AN = (move[1][1])[:2]
        goal_AN = (move[1][1])[2:4]

        return "Died", start_AN, goal_AN


def AN_to_coords(AN, board_points):
    """Converts algebraic notation into real world ``x, y, z`` coordinates

    Attributes:
        * ``AN``: The algebraic notation from the game engine
        * ``board_points``: The collected coordinates on the FRANKA frame of the corners of the board

    """

    A1 = board_points[0]
    A8 = board_points[1]
    H8 = board_points[2]

    x = (H8[0] - A1[0])/8 # square width in FRANKA units
    y = (A8[1] - A1[1])/8  # square length in FRANKA units

    # find coordinates of each AN
    letters = dict([('a', A1[0] + 0.5*x), ('b', A1[0] + 1.5*x), ('c', A1[0] + 2.5*x), ('d', A1[0]+ 3.5*x), ('e', A1[0] + 4.5*x),
                    ('f', A1[0] + 5.5*x), ('g', A1[0] + 6.5*x), ('h', A1[0] + 7.5*x)])
    numbers = dict([('1', A1[1] + 0.5*y), ('2', A1[1] + 1.5*y), ('3', A1[1] + 2.5*y), ('4', A1[1] + 3.5*y), ('5', A1[1] + 4.5*y),
                    ('6', A1[1] + 5.5*y), ('7', A1[1] + 6.5*y), ('8', A1[1] + 7.5*y)])

    # selecting the location
    letter = AN[0]
    number = AN[1]
    x = numbers[number]
    y = letters[letter]
    z = 0

    coord = [x, y, z]

    return coord

# def gripper_height():
    # Finding the gripper height depending on game engine output
    # dictionary of heights for each piece


def print_actions(dead_status, trajectories):
    """Prints the separate trajectories surrounding gripping and ungripping"""

    if dead_status == 'Died':
        print('\n-------------------------------------------------------------------\nMovements:\n')
        print('Trajectory 1:\n', trajectories[1])
        print('grip')
        print('Trajectory 2:\n', trajectories[2])
        print('ungrip')
        print('Trajectory 3:\n', trajectories[3])

    elif dead_status == 'None died':
        print('\n-------------------------------------------------------------------\nMovements:\n')
        print('Trajectory 1:\n', trajectories[1])
        print('grip')
        print('Trajectory 2:\n', trajectories[2])
        print('ungrip')
        print('Trajectory 3:\n', trajectories[3])
        print('grip')
        print('Trajectory 4:\n', trajectories[4])
        print('ungrip')
        print('Trajectory 5:\n', trajectories[5])


def intermediate_coords(rest, start, goal, hover, dead_zone):
    """Generate the intermediate positions of FRANKA along the path

    The attributes are the 5 known locations of FRANKA:
        * ``rest``
        * ``start``
        * ``goal``
        * ``hover``
        * ``dead_zone``

    Returns the desired intermediate positions:
        * ``rest_h``, rest hover
        * ``start_h``, start hover
        * ``goal_h``, goal hover
        * ``deadzone_h``, deadzone hover
    """

    rest_h = [rest[0], rest[1], hover]
    start_h = [start[0], start[1], hover]
    goal_h = [goal[0], goal[1], hover]
    deadzone_h = [dead_zone[0], dead_zone[1], hover]

    return rest_h, start_h, goal_h, deadzone_h


def create_line(a, b, n):
    """Generates a list of coordinates ``n`` metres apart between 2 points ``a`` and ``b`` in space."""

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
    return line


def start_path(dead_status, rest, start_h, goal_h, n):
    """ Generate the path from the rest position to the first location, based on the ``dead_status`` """

    if dead_status == 'None died':
        l0 = create_line(rest, start_h, n)
    elif dead_status == 'Died':
        l0 = create_line(rest, goal_h, n)
    return [l0]


def move_path(rest, start, start_h, goal, goal_h, n):
    """ Generate the path between the start hover position and return to rest position"""

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


def dead_path(goal, goal_h, dead_zone, deadzone_h, start_h, n):
    """ Generate the path between the goal hover position and the start hover position to remove a piece from the board and drop it into the deadzone """

    l1 = create_line(goal_h, goal, n)
    # grip
    l2 = create_line(goal, goal_h, n)
    l3 = create_line(goal_h, deadzone_h, n)
    l4 = create_line(deadzone_h, dead_zone, n)
    # ungrip
    l5 = create_line(dead_zone, deadzone_h, n)
    l6 = create_line(deadzone_h, start_h, n)

    # Remove duplicates
    line_list = [l1, l2, l3, l4, l5, l6]
    lines = []
    for line in line_list:
        line = np.delete(line, 0, 0)
        lines.append(line)

    return lines


def generate_trajectory(line_list, dead_status):
    """Joins together the separate lines to make a full trajectory

    Attributes:
        *``line_list``: A list of lines, each of which is made up of many x,y,z coordinates
        *``dead_status``: Determines what trajectory to generate, based on whether a piece has died

    Returns:
        *``trajectory``: A list of coordinates starting and ending at the rest position
        *``trajectory_n``: A trajectory of points between either a rest position and a gripping action, or 2 gripping actions
    """

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
    """Splits the data into ``x, y, z``"""

    x = [(trajectory[i][0]) for i in range(len(trajectory))]
    y = [(trajectory[i][1]) for i in range(len(trajectory))]
    z = [(trajectory[i][2]) for i in range(len(trajectory))]
    return x, y, z


def data_interpolation(trajectory, x_sample, y_sample, z_sample):
    """Performs interpolation to find smoothed trajectory"""

    tck, u = interpolate.splprep([x_sample, y_sample, z_sample], s=0.01)  # s is amount of smoothness
    x_knots, y_knots, z_knots = interpolate.splev(tck[0], tck)
    u_fine = np.linspace(0, 1, len(trajectory))
    x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)
    return x_fine, y_fine, z_fine, x_knots, y_knots, z_knots


def smooth(trajectory):
    """ Generates a smooth version of the inputted trajectory

    Returns:
        * ``smooth_trajectory``: The smoothed trajectory as a list of coordinates
        * ``x_sample, y_sample, z_sample`` : ``x, y, z`` coordinates of the inputted trajectory
        * ``x_knots, y_knots, z_knots``: ``x, y, z`` coordinates of the knot points
        * ``x_smooth, y_smooth, z_smooth``: ``x, y, z`` coordinates of the smoothed trajectory
    """

    x_sample, y_sample, z_sample = data_split(trajectory)
    x_fine, y_fine, z_fine, x_knots, y_knots, z_knots = data_interpolation(trajectory, x_sample, y_sample, z_sample)

    x_fine = [decimal.Decimal(i) for i in x_fine]
    x_fine = [float(round(i, 3)) for i in x_fine]

    y_fine = [decimal.Decimal(i) for i in y_fine]
    y_fine = [float(round(i,3)) for i in y_fine]

    z_fine = [decimal.Decimal(i) for i in z_fine]
    z_fine = [float(round(i, 3)) for i in z_fine]

    smooth_trajectory = zip(x_fine, y_fine, z_fine)
    smooth_trajectory = [list(elem) for elem in smooth_trajectory]
    x_smooth, y_smooth, z_smooth = data_split(smooth_trajectory)


    return smooth_trajectory, x_sample, y_sample, z_sample, x_knots, y_knots, z_knots, x_smooth, y_smooth, z_smooth


def generate_vectors(trajectory):
    """ Creates motion vectors between every point on a given trajectory"""

    vectors = []
    for i in range(1, len(trajectory)):
        previous = trajectory[i-1]
        current = trajectory[i]

        vector = [previous[0] - current[0], previous[1] - current[1], previous[2] - current[2]]  # x, y, z

        vectors.append(vector)

    return vectors


def plot(x_sample, y_sample, z_sample, x_knots, y_knots, z_knots, x_smooth, y_smooth, z_smooth):
    """Plot the initial trajectory and its smoothed version"""

    fig = plt.figure()
    ax3d = fig.add_subplot(111, projection='3d')
    ax3d.plot(x_sample, y_sample, z_sample, 'r')  # plotting the angled points
    # ax3d.plot(x_knots, y_knots, z_knots, 'go')  # plotting the curve knots
    ax3d.plot(x_smooth, y_smooth, z_smooth, 'g')  # plotting the smoothed trajectory
    plt.show()


def gaussian(v, x, mu, sig):
    return v * (exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.))))


def generate_speeds(max_v, trajectory):
    """Generates the velocity profile as a bell curve"""

    max_position = len(trajectory) / 2
    spread = 500  # how wide the bell curve is
    speeds = [max_v * exp(-((i - max_position) ** 2.) / (2. * (spread ** 2.))) for i in range(len(trajectory))]

    return speeds

if __name__ == '__main__':
    # output([("r", "b4"),("r", "a1a2")] ,visual_flag=True) # test for death
    output([("r", "a1a2")] ,visual_flag=True)  # test for standard move
