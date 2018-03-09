from __future__ import print_function
from numpy import arange, sqrt, linspace, zeros, concatenate
#from builtins import input
from numpy import arange
from matplotlib.pyplot import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # for: fig.gca(projection = '3d')
import decimal

# Python 2/3 raw input correction
try:
   input = raw_input
except NameError:
   pass


class MotionPlanning:
    def __init__(self):
        """Gets the location of each corner of the board and stores them"""

        print("For the following commands, place the FRANKA end effector in the centre of the relevant square\n")

        valid = input("Move the arm to A1 corner")
        if valid == "":
            pass
        try:
            A1 = arm.get_position()
            print('We are using actual Franka coordinates')
        except:
            A1 = [0.730, -0.269, 0.076]

        valid = input("Move the arm to A8 corner")
        if valid == "":
            pass
        try:
            A8 = arm.get_position()
        except:
            A8 = [0.265, -0.266, 0.069]

        valid = input("Move the arm to H8 corner")
        if valid == "":
            pass
        try:
            H8 = arm.get_position()
        except:
            H8 = [0.303, 0.197, 0.071]

        valid = input("Move the arm to H1 corner")
        if valid == "":
            pass
        try:
            H1 = arm.get_position()
        except:
            H1 = [0.741, 0.163, 0.077]

        self.board_points = [A1, A8, H8, H1]
        self.A1 = A1
        self.A8 = A8
        self.H8 = H8
        self.H1 = H1

        # Find the maximum z value of the board
        max_z = max(A1[2], A8[2], H1[2], H8[2])
        self.board_z_max = max_z

        # Find x axis vector
        x_vector_1 = [(A8[0] - H8[0]), (A8[1] - H8[1]), max_z]
        x_vector_2 = [(A1[0] - H1[0]), (A1[1] - H1[1]), max_z]
        x_vector = [sum(x)/2 for x in zip(x_vector_1, x_vector_2)]
        self.x_unit_vector = [coord/8 for coord in x_vector]

        # Find y axis vector
        y_vector_1 = [(H1[0] - H8[0]), (H1[1] - H8[1]), max_z]
        y_vector_2 = [(A1[0] - A8[0]), (A1[1] - A8[1]), max_z]
        y_vector = [sum(x)/2 for x in zip(y_vector_1, y_vector_2)]
        self.y_unit_vector = [coord/8 for coord in y_vector]

        # Find hover height
        self.hover_height = max_z + 0.2  # hard coded hover height

        # Find location of deadzone
        deadzone_x_vector = [-(i * 4) for i in self.x_unit_vector]
        deadzone_y_vector = [i * 4 for i in self.y_unit_vector]
        deadzone_z = max_z + 0.1 # hardcoded z coordinate of deadzone
        deadzone = [sum(i) for i in zip(self.H8, deadzone_x_vector, deadzone_y_vector)]
        deadzone[2] = deadzone_z
        self.deadzone = deadzone

        # Find the location of the rest position
        rest_x_vector = [i * 4 for i in self.x_unit_vector]
        rest_y_vector = [-(i * 3) for i in self.y_unit_vector]
        rest_z = max_z + 0.9  # hardcoded z coordinate of the rest position
        rest = [sum(i) for i in zip(self.H8, rest_x_vector, rest_y_vector)]
        rest[2] = rest_z
        self.rest = rest

        # Find coordinates of each square
        self.letters = dict([('h', [i * 0.5 for i in self.x_unit_vector]),
                        ('g', [i * 1.5 for i in self.x_unit_vector]),
                        ('f', [i * 2.5 for i in self.x_unit_vector]),
                        ('e', [i * 3.5 for i in self.x_unit_vector]),
                        ('d', [i * 4.5 for i in self.x_unit_vector]),
                        ('c', [i * 5.5 for i in self.x_unit_vector]),
                        ('b', [i * 6.5 for i in self.x_unit_vector]),
                        ('a', [i * 7.5 for i in self.x_unit_vector])
                        ])

        self.numbers = dict([('8', [i * 0.5 for i in self.y_unit_vector]),
                        ('7', [i * 1.5 for i in self.y_unit_vector]),
                        ('6', [i * 2.5 for i in self.y_unit_vector]),
                        ('5', [i * 3.5 for i in self.y_unit_vector]),
                        ('4', [i * 4.5 for i in self.y_unit_vector]),
                        ('3', [i * 5.5 for i in self.y_unit_vector]),
                        ('2', [i * 6.5 for i in self.y_unit_vector]),
                        ('1', [i * 7.5 for i in self.y_unit_vector])
                        ])

    def make_move(self, move, visual_flag=False):

        # Extract information from output of game engine
        if len(move) == 1:

            start_AN = (move[0][1])[:2]
            goal_AN = (move[0][1])[2:4]
            dead_status = "None died"

        elif len(move) == 2:

            start_AN = (move[1][1])[:2]
            goal_AN = (move[1][1])[2:4]
            dead_status = "Died"

        # Convert ANs to coordinates
        start = self.AN_to_coords(start_AN)
        goal = self.AN_to_coords(goal_AN)

        # Generate the intermediate positions of the path
        start_h = [start[0], start[1], self.hover_height]
        goal_h = [goal[0], goal[1], self.hover_height]
        deadzone_h = [self.deadzone[0], self.deadzone[1], self.hover_height]

        # Join up these positions to create a path
        dx = 0.005  # distance between points

        if dead_status == 'None died':
            path = [self.rest, start_h, start, start_h, goal_h, goal, goal_h, self.rest]

            line_1 = discretise(self.rest, start_h, dx)
            line_2 = discretise(start_h, start, dx)
            line_3 = discretise(start, start_h, dx)
            line_4 = discretise(start_h, goal_h, dx)
            line_5 = discretise(goal_h, goal, dx)
            line_6= discretise(goal, goal_h, dx)
            line_7= discretise(goal_h, self.rest, dx)

            line_list = [line_1, line_2, line_3, line_4, line_5, line_6, line_7]

        elif dead_status == 'Died':
            path = [self.rest, goal_h, goal, goal_h, deadzone_h, self.deadzone, deadzone_h, start_h, start, start_h, goal_h, goal, goal_h, self.rest]

            line_1 = discretise(self.rest, goal_h, dx)
            line_2 = discretise(goal_h, deadzone_h, dx)
            line_3 = discretise(self.deadzone, deadzone_h, dx)
            line_4 = discretise(deadzone_h, start_h, dx)
            line_5 = discretise(start_h, start, dx)
            line_6 = discretise(start, start_h, dx)
            line_7 = discretise(start_h, goal_h, dx)
            line_8 = discretise(goal_h, goal, dx)
            line_9 = discretise(goal, goal_h, dx)
            line_10 = discretise(goal_h, self.rest, dx)

            line_list = [line_1, line_2, line_3, line_4, line_5, line_6, line_7, line_8, line_9, line_10]

        lines = []
        for i in range(len(line_list)-1):
            if line_list[i][-1][0] == line_list[i+1][0][0]:
                print('Dingity dangit theyre the same')
                # line = old line with the last number removed
                lines.append(line_list[i][:-1])
            else:
                lines.append(line_list[i])
        lines.append(line_list[-1])

        if dead_status == 'None died':

            # Forming the 3 movements that surround the gripping and ungripping
            line_list_1 = (lines[0], lines[1])
            line_list_2 = (lines[2], lines[3], lines[4])
            line_list_3 = (lines[5], lines[6])

            trajectory_1 = concatenate(line_list_1, axis=0)
            trajectory_2 = concatenate(line_list_2, axis=0)
            trajectory_3 = concatenate(line_list_3, axis=0)
            trajectories = [trajectory_1, trajectory_2, trajectory_3]


        elif dead_status == 'Died':

            # Forming the 2 movements that surround the gripping and ungripping to remove a dead piece
            line_list_1 = (lines[0], lines[1])
            line_list_2 = (lines[2], lines[3], lines[4])
            line_list_3 = (lines[5], lines[6], lines[7])
            line_list_4 = (lines[8], lines[9], lines[10])
            line_list_5 = (lines[11], lines[12])

            trajectory_1 = concatenate(line_list_1, axis=0)
            trajectory_2 = concatenate(line_list_2, axis=0)
            trajectory_3 = concatenate(line_list_3, axis=0)
            trajectory_4 = concatenate(line_list_4, axis=0)
            trajectory_5 = concatenate(line_list_5, axis=0)
            trajectories = [trajectory_1, trajectory_2, trajectory_3, trajectory_4, trajectory_5]

        # Smooth corners
        steps = 12  # must be an even number

        trajectory_prev = trajectory_1

        for i in range(5):
            trajectory_1_x = [item[0] for item in trajectory_1]
            trajectory_1_y = [item[1] for item in trajectory_1]
            trajectory_1_z = [item[2] for item in trajectory_1]

            x_tortoise = trajectory_1_x[:-steps]  # remove last few coords
            x_hare = trajectory_1_x[steps:]  # first last few coords
            x_smooth_path = [(sum(i) / 2) for i in zip(x_tortoise, x_hare)]  # average them

            y_tortoise = trajectory_1_y[:-steps]  # remove last few coords
            y_hare = trajectory_1_y[steps:]  # remove first few coords
            y_smooth_path = [(sum(i) / 2) for i in zip(y_tortoise, y_hare)]

            z_tortoise = trajectory_1_z[:-steps]  # remove last few coords
            z_hare = trajectory_1_z[steps:]  # remove first few coords
            z_smooth_path = [(sum(i) / 2) for i in zip(z_tortoise, z_hare)]

            smooth_path = np.array([list(i) for i in zip(x_smooth_path, y_smooth_path, z_smooth_path)])
            # append first 6 coords in trajectory_1 to the smooth_path
            print(trajectory_1[:(steps/2)])
            print(smooth_path)

            smooth_path = np.concatenate((trajectory_1[:(steps/2)], smooth_path, trajectory_1[-(steps/2):]), axis=0)
            trajectory_1 = smooth_path





        # for i in range(40):
        #     print(trajectory_prev[i], "    ", trajectory_1[i])


        # # Removing excess precision
        # x = [decimal.Decimal(i) for i in x_raw]
        # x = [float(round(i, 3)) for i in x]
        #
        # y = [decimal.Decimal(i) for i in y_raw]
        # y = [float(round(i, 3)) for i in y]
        #
        # z = [decimal.Decimal(i) for i in z_raw]
        # z = [float(round(i, 3)) for i in z]

        if visual_flag:

            # Separate into xyz
            x = [coord[0] for coord in path]
            y = [coord[1] for coord in path]
            z = [coord[2] for coord in path]

            # x_smooths = []
            # y_smooths = []
            # z_smooths = []
            # for path in smooth_paths:
            #     x_smooth = [coord[0] for coord in path]
            #     y_smooth = [coord[1] for coord in path]
            #     z_smooth = [coord[2] for coord in path]
            #     x_smooths = x_smooths + x_smooth
            #     y_smooths = y_smooths + y_smooth
            #     z_smooths = z_smooths + z_smooth


            # getting board points
            board_x = [coord[0] for coord in self.board_points]
            board_y = [coord[1] for coord in self.board_points]
            board_z = [coord[2] for coord in self.board_points]


            board_x.append(self.board_points[0][0])
            board_y.append(self.board_points[0][1])
            board_z.append(self.board_points[0][2])

            # plotting the board
            fig = plt.figure()
            ax3d = fig.add_subplot(111, projection='3d')

            ax3d.plot(board_x, board_y, board_z, 'b')  # plot the board
            ax3d.plot(x, y, z, 'r')  # plot path
            ax3d.plot([self.rest[0]], [self.rest[1]], [self.rest[2]], 'g*')
            ax3d.plot([self.deadzone[0]], [self.deadzone[1]], [self.deadzone[2]], 'g*')
            ax3d.plot(smooth_path[:,0], smooth_path[:,1], smooth_path[:,2], 'b*')

            plt.show()

        print(path)
        return path

    def AN_to_coords(self, AN):
        """Converts algebraic notation into real world ``x, y, z`` coordinates"""

        # selecting the location
        letter = AN[0]
        number = AN[1]
        x = self.numbers[number]
        y = self.letters[letter]
        xy_coord = [sum(i) for i in zip(self.H8, x, y)]
        xy_coord[2] = self.board_z_max
        coord = xy_coord

        return coord



# def discretised():
#     """Discretises a path"""
#
#     discretised_path = []
#
#     resolution = 0.001
#
#     for path_n in paths:
#         d_path_x = arange(path_n[0][0], path_n[-1][0], resolution)
#         print('dpath', len(d_path_x))
#         d_path_y = arange(path_n[0][1], path_n[-1][1], resolution)
#         d_path_z = arange(path_n[0][2], path_n[-1][2], resolution)
#         d_path = [d_path_x, d_path_y, d_path_z]
#
#         discretised_paths.append(d_path)
#
#     # Smooth corners
#     roundness = 5
#     smooth_paths = []
#     for path_n in discretised_paths:
#         print(len(path_n))
#         tortoise = path_n[:-roundness]  # remove last few coords
#         print(tortoise)
#         hare = path_n[roundness:]  # remove last few coords
#         smooth_path = [sum(x) / 2 for x in zip(tortoise, hare)]
#         smooth_paths.append(smooth_path)
#
#     print(smooth_paths)
#     print(len(discretised_paths))





def discretise(a, b, n):
    """Generates a list of coordinates ``n`` metres apart between 2 points ``a`` and ``b`` in space."""

    vector = [b[0] - a[0], b[1] - a[1], b[2] - a[2]]  # vector from b to a
    distance = sqrt(sum(i ** 2 for i in vector))

    # number of points on line
    i = int(distance/n)

    # pre-allocate space
    line = zeros((i, 3))

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


FRANKA = MotionPlanning()
FRANKA.make_move([("r", "a1a6")], visual_flag=True)