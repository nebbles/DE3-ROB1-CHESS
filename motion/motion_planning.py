#from franka.franka_control_ros import FrankaRos
from numpy import mean
from matplotlib.pyplot import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # for: fig.gca(projection = '3d')
import decimal

# TODO: Add pyhton 2 compatibility
# TODO: install ros packages to run with Franka
# TODO: Sort out AN to real life conversion

class MotionPlanning:
    def __init__(self):
        """Gets the location of each corner of the board and stores them"""

        #arm = FrankaRos()  # debug=True)

        print("For the following commands, place the FRANKA end effector in the centre of the relevant square\n")

        valid = input("Move the arm to A1 corner")
        if valid == "":
            pass
        try:
            A1 = arm.get_position()
            print('We are using actual Franka coordinates')
        except:
            A1 = [0.501, 0.199, 0.048]

        valid = input("Move the arm to A8 corner")
        if valid == "":
            pass
        try:
            A8 = arm.get_position()
        except:
            A8 = [0.739, 0.14, 0.0344]

        valid = input("Move the arm to H8 corner")
        if valid == "":
            pass
        try:
            H8 = arm.get_position()
        except:
            H8 = [0.71, -0.176, 0]

        valid = input("Move the arm to H1 corner")
        if valid == "":
            pass
        try:
            H1 = arm.get_position()
        except:
            H1 = [0.468, -0.138, -0.0156]

        self.board_points = [A1, A8, H8, H1]

        # Find hover height
        board_points_z = [corner[2] for corner in self.board_points]
        self.hover_height = mean(board_points_z) + 0.2  # hard coded hover height

        # Find location of deadzone
        deadzone_x = mean([A1[0], A8[0]]) + 0.15 # hard coded x coordinate of deadzone
        deadzone_y = (A8[1] - A1[1])/2 # exact centre of board
        deadzone_z = mean(board_points_z) + 0.1 # hardcoded z coordinate of deadzone
        self.deadzone = [deadzone_x, deadzone_y, deadzone_z]

        # Find the location of the rest position
        rest_x = (H1[0] - A1[0])/2
        rest_y = (A8[1] - A1[1])/2
        rest_z = mean(board_points_z) + 0.9
        self.rest = [rest_x, rest_y, rest_z]

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

        x_raw = [coord[0] for coord in path]
        y_raw = [coord[1] for coord in path]
        z_raw = [coord[2] for coord in path]

        # Removing excess precision
        x = [decimal.Decimal(i) for i in x_raw]
        x = [float(round(i, 3)) for i in x]

        y = [decimal.Decimal(i) for i in y_raw]
        y = [float(round(i, 3)) for i in y]

        z = [decimal.Decimal(i) for i in z_raw]
        z = [float(round(i, 3)) for i in z]

        if visual_flag:
            fig = plt.figure()
            ax3d = fig.add_subplot(111, projection='3d')

            board_x = [coord[0] for coord in self.board_points]
            board_y = [coord[1] for coord in self.board_points]
            board_z = [coord[2] for coord in self.board_points]

            board_x.append(self.board_points[0][0])
            board_y.append(self.board_points[0][1])
            board_z.append(self.board_points[0][2])

            ax3d.plot(board_x, board_y, board_z, 'b')  # plot the board
            ax3d.plot(x, y, z, 'r')  # plot path
            plt.show()

        print(path)
        return path

    def AN_to_coords(self, AN):
        """Converts algebraic notation into real world ``x, y, z`` coordinates

        Attributes:
            * ``AN``: The algebraic notation from the game engine
            * ``board_points``: The collected coordinates on the FRANKA frame of the corners of the board

        """

        # Location of corners of the board
        A1 = self.board_points[0]
        A8 = self.board_points[1]
        H8 = self.board_points[2]
        H1 = self.board_points[3]


        x1 = abs((H1[0] - A1[0]) / 8)
        x2 = abs((H8[0] - A8[0]) / 8)
        x = mean([x1, x2])  # square width in FRANKA units
        y1 = abs((A8[1] - A1[1]) / 8)
        y2 = abs((H8[1] - H1[1]) / 8)
        y = mean([y1, y2])  # square length in FRANKA units

        # find coordinates of each AN TODO CORRECT THIS!!!

        # is x axis in the given x, y z coords, the same as the x axis of the board?? CRITICAL
        letters = dict([('a', A1[0] + 0.5 * x), ('b', A1[0] + 1.5 * x), ('c', A1[0] + 2.5 * x), ('d', A1[0] + 3.5 * x),
                        ('e', A1[0] + 4.5 * x),
                        ('f', A1[0] + 5.5 * x), ('g', A1[0] + 6.5 * x), ('h', A1[0] + 7.5 * x)])
        numbers = dict([('1', A1[1] + 0.5 * y), ('2', A1[1] + 1.5 * y), ('3', A1[1] + 2.5 * y), ('4', A1[1] + 3.5 * y),
                        ('5', A1[1] + 4.5 * y),
                        ('6', A1[1] + 5.5 * y), ('7', A1[1] + 6.5 * y), ('8', A1[1] + 7.5 * y)])

        # selecting the location
        letter = AN[0]
        number = AN[1]
        x = numbers[number]
        y = letters[letter]
        z = 0

        coord = [x, y, z]

        return coord
