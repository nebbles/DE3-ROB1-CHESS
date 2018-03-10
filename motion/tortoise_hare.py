from __future__ import print_function
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # for: fig.gca(projection = '3d')

try:  # Python 2/3 raw input correction
    input = raw_input
except NameError:
    pass


class MotionPlanner:
    def __init__(self, arm_object, visual=False):
        """Gets the location of each corner of the board and stores them"""
        self.visual = visual
        self.dx = 0.005

        print("For the following commands, place the FRANKA end effector in the centre of the "
              "relevant square\n")

        input("Move the arm to A1 corner")
        try:
            a1 = arm_object.get_position()
            print('We are using actual Franka coordinates')
        except:
            a1 = [0.730, -0.269, 0.076]

        input("Move the arm to A8 corner")
        try:
            a8 = arm_object.get_position()
        except:
            a8 = [0.265, -0.266, 0.069]

        input("Move the arm to H8 corner")
        try:
            h8 = arm_object.get_position()
        except:
            h8 = [0.303, 0.197, 0.071]

        input("Move the arm to H1 corner")
        try:
            h1 = arm_object.get_position()
        except:
            h1 = [0.741, 0.163, 0.077]

        self.board = [a1, a8, h8, h1]
        self.H8 = h8

        # Find the maximum z value of the board
        self.board_z_max = max(a1[2], a8[2], h1[2], h8[2])

        # Find x axis vector
        x_vector_1 = [(a8[0] - h8[0]), (a8[1] - h8[1]), self.board_z_max]
        x_vector_2 = [(a1[0] - h1[0]), (a1[1] - h1[1]), self.board_z_max]
        x_vector = [sum(x)/2 for x in zip(x_vector_1, x_vector_2)]
        self.x_unit_vector = [coord/8 for coord in x_vector]

        # Find y axis vector
        y_vector_1 = [(h1[0] - h8[0]), (h1[1] - h8[1]), self.board_z_max]
        y_vector_2 = [(a1[0] - a8[0]), (a1[1] - a8[1]), self.board_z_max]
        y_vector = [sum(x)/2 for x in zip(y_vector_1, y_vector_2)]
        self.y_unit_vector = [coord/8 for coord in y_vector]

        # Find hover height
        self.hover_height = self.board_z_max + 0.2  # hard coded hover height

        # Find location of dead zone
        dead_zone_x_vector = [-(i * 4) for i in self.x_unit_vector]
        dead_zone_y_vector = [i * 4 for i in self.y_unit_vector]
        dead_zone_z = self.board_z_max + 0.2  # hardcoded z coordinate of dead zone
        dead_zone = [sum(i) for i in zip(self.H8, dead_zone_x_vector, dead_zone_y_vector)]
        dead_zone[2] = dead_zone_z
        self.dead_zone = dead_zone

        # Find the location of the rest position
        rest_x_vector = [i * 4 for i in self.x_unit_vector]
        rest_y_vector = [-(i * 3) for i in self.y_unit_vector]
        rest_z = self.board_z_max + 0.9  # hardcoded z coordinate of the rest position
        rest = [sum(i) for i in zip(self.H8, rest_x_vector, rest_y_vector)]
        rest[2] = rest_z
        self.rest = rest

        # Find coordinates of each square
        self.letter_dict = dict([('h', [i * 0.5 for i in self.x_unit_vector]),
                                 ('g', [i * 1.5 for i in self.x_unit_vector]),
                                 ('f', [i * 2.5 for i in self.x_unit_vector]),
                                 ('e', [i * 3.5 for i in self.x_unit_vector]),
                                 ('d', [i * 4.5 for i in self.x_unit_vector]),
                                 ('c', [i * 5.5 for i in self.x_unit_vector]),
                                 ('b', [i * 6.5 for i in self.x_unit_vector]),
                                 ('a', [i * 7.5 for i in self.x_unit_vector])
                                 ])

        self.number_dict = dict([('8', [i * 0.5 for i in self.y_unit_vector]),
                                 ('7', [i * 1.5 for i in self.y_unit_vector]),
                                 ('6', [i * 2.5 for i in self.y_unit_vector]),
                                 ('5', [i * 3.5 for i in self.y_unit_vector]),
                                 ('4', [i * 4.5 for i in self.y_unit_vector]),
                                 ('3', [i * 5.5 for i in self.y_unit_vector]),
                                 ('2', [i * 6.5 for i in self.y_unit_vector]),
                                 ('1', [i * 7.5 for i in self.y_unit_vector])
                                 ])

    def generate_chess_motion(self, move):
        """Called after chess move has been determined"""
        # Extract information from output of game engine
        start_an = (move[0][1])[:2]
        goal_an = (move[0][1])[2:4]

        # Convert ANs to coordinates
        start = self.an_to_coords(start_an)
        goal = self.an_to_coords(goal_an)

        # Generate the intermediate positions of the path
        start_hover = [start[0], start[1], self.hover_height]
        goal_hover = [goal[0], goal[1], self.hover_height]
        dead_zone_hover = [self.dead_zone[0], self.dead_zone[1], self.hover_height]

        if len(move) == 1:  # NO PIECE DIED
            dead_status = False
            path = [self.rest, start_hover, start, start_hover, goal_hover, goal, goal_hover,
                    self.rest]

        elif len(move) == 2:  # A PIECE HAS DIED
            dead_status = True
            path = [self.rest, goal_hover, goal, goal_hover, dead_zone_hover, self.dead_zone,
                    dead_zone_hover, start_hover, start, start_hover, goal_hover, goal,
                    goal_hover, self.rest]

        line_list = []
        for i in range(len(path) - 1):
            line_list.append(self.discretise(path[i], path[i + 1], self.dx))

        lines = []
        for i in range(len(line_list)-1):
            if line_list[i][-1][0] == line_list[i+1][0][0]:
                print('Dingity dangit theyre the same')
                # line = old line with the last number removed
                lines.append(line_list[i][:-1])
            else:
                lines.append(line_list[i])
        lines.append(line_list[-1])

        if not dead_status:

            # Forming the 3 movements that surround the gripping and ungripping
            line_list_1 = (lines[0], lines[1])
            line_list_2 = (lines[2], lines[3], lines[4])
            line_list_3 = (lines[5], lines[6])

            trajectory_1 = np.concatenate(line_list_1, axis=0)
            trajectory_2 = np.concatenate(line_list_2, axis=0)
            trajectory_3 = np.concatenate(line_list_3, axis=0)
            trajectories = [trajectory_1, trajectory_2, trajectory_3]


        elif dead_status:

            # Forming the 2 movements that surround the gripping and ungripping to remove a dead piece
            line_list_1 = (lines[0], lines[1])
            line_list_2 = (lines[2], lines[3], lines[4])
            line_list_3 = (lines[5], lines[6], lines[7])
            line_list_4 = (lines[8], lines[9], lines[10])
            line_list_5 = (lines[11], lines[12])

            trajectory_1 = np.concatenate(line_list_1, axis=0)
            trajectory_2 = np.concatenate(line_list_2, axis=0)
            trajectory_3 = np.concatenate(line_list_3, axis=0)
            trajectory_4 = np.concatenate(line_list_4, axis=0)
            trajectory_5 = np.concatenate(line_list_5, axis=0)
            trajectories = [trajectory_1, trajectory_2, trajectory_3, trajectory_4, trajectory_5]

        # Smooth corners
        steps = 12  # must be an even number


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

        if self.visual:

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
            board_x = [coord[0] for coord in self.board]
            board_y = [coord[1] for coord in self.board]
            board_z = [coord[2] for coord in self.board]


            board_x.append(self.board[0][0])
            board_y.append(self.board[0][1])
            board_z.append(self.board[0][2])

            # plotting the board
            fig = plt.figure()
            ax3d = fig.add_subplot(111, projection='3d')

            ax3d.plot(board_x, board_y, board_z, 'b')  # plot the board
            ax3d.plot(x, y, z, 'r')  # plot path
            ax3d.plot([self.rest[0]], [self.rest[1]], [self.rest[2]], 'g*')
            ax3d.plot([self.dead_zone[0]], [self.dead_zone[1]], [self.dead_zone[2]], 'g*')
            ax3d.plot(smooth_path[:,0], smooth_path[:,1], smooth_path[:,2], 'b*')

            plt.show()

        print(path)
        return path

    def an_to_coords(self, AN):
        """Converts algebraic notation into real world ``x, y, z`` coordinates"""

        # selecting the location

        # split AN location
        letter = AN[0]
        number = AN[1]

        # lookup in dictionary
        x_in_chess = self.number_dict[number]
        y_in_chess = self.letter_dict[letter]

        # move by dims_in_chess from the H8 coordinate
        coord = [sum(i) for i in zip(self.H8, x_in_chess, y_in_chess)]
        # overwrite z coordinate with predetermined z height
        coord[2] = self.board_z_max
        return coord

    @staticmethod
    def discretise(point_1, point_2, dx):
        """Generates a list of coordinates ``n`` metres apart between 2 points ``a`` and ``b`` in
        space."""

        # create vector from point_1 to point_2
        vector = [point_2[0] - point_1[0], point_2[1] - point_1[1], point_2[2] - point_1[2]]
        distance = np.sqrt(sum(i ** 2 for i in vector))

        # number of points on line
        i = int(distance / dx)

        # discretise by creating new 1d array
        line_x = np.linspace(point_1[0], point_2[0], i)
        line_y = np.linspace(point_1[1], point_2[1], i)
        line_z = np.linspace(point_1[2], point_2[2], i)
        line = np.transpose(np.vstack((line_x, line_y, line_z)))
        return line


if __name__ == '__main__':
    arm = None
    FRANKA = MotionPlanner(arm, visual=True)
    FRANKA.generate_chess_motion([("r", "a1a6")])
