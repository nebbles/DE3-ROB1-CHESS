from __future__ import print_function
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
        if dead_status == 'None died':
            path = [self.rest, start_h, start, start_h, goal_h, goal, goal_h, self.rest]

        elif dead_status == 'Died':
            path = [self.rest, goal_h, goal, goal_h, deadzone_h, self.deadzone, deadzone_h, start_h, start, start_h, goal_h, goal, goal_h, self.rest]

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
            # ax3d.plot(z_smooths, y_smooths, z_smooths, 'b')

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

