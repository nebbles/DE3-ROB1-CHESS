from __future__ import print_function
import numpy as np
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # for: fig.gca(projection = '3d')
import time
import rospy
from std_msgs.msg import Float64MultiArray
import thread

glob_curr_pos_x = None
glob_curr_pos_y = None
glob_curr_pos_z = None

try:  # Python 2/3 raw input correction
    input = raw_input
except NameError:
    pass


class MotionPlanner:
    def __init__(self, arm_object, visual=False, manual_calibration=False, debug=False):
        """Gets the location of each corner of the board and stores them"""
        self.visual = visual
        self.dx = 0.005
        self.debug = debug

        if manual_calibration:
            print("For the following commands, place the FRANKA end effector in the centre of the "
                  "relevant square\n")

            input("Move the arm to A1 corner")
            a1 = arm_object.get_position()

            input("Move the arm to A8 corner")
            a8 = arm_object.get_position()

            input("Move the arm to H8 corner")
            h8 = arm_object.get_position()

            input("Move the arm to H1 corner")
            h1 = arm_object.get_position()

        else:
            # FIRST VALUES
            # a1 = [0.730, -0.269, 0.076]
            # a8 = [0.265, -0.266, 0.069]
            # h8 = [0.303, 0.197, 0.071]
            # h1 = [0.741, 0.163, 0.077]
            # NEW VALUES WED 13 MARCH
            a1 = [0.757,  -0.257,  -0.04]
            a8 = [0.3076, -0.2515, -0.04]
            h8 = [0.326,   0.2125, -0.04]
            h1 = [0.7702,  0.1956, -0.04]

        # else:  # from Paolo's collected coords
        #     a1_inner = 
        #     a8_ inner = 
        #     h8_inner = 
        #     h1_inner = 

        self.board = [a1, a8, h8, h1]
        self.H8 = h8

        # Find the maximum z value of the board
        self.board_z_max = -0.02

        # Find x axis vector
        x_vector_1 = [(a8[0] - h8[0]), (a8[1] - h8[1]), self.board_z_max]
        x_vector_2 = [(a1[0] - h1[0]), (a1[1] - h1[1]), self.board_z_max]
        x_vector = [sum(x) / 2 for x in zip(x_vector_1, x_vector_2)]
        self.x_unit_vector = [coord / 8 for coord in x_vector]

        # from Paolo's collected coords
        # x_vector_1 = [(a8_inner[0] - h8_inner[0]), (a8_inner[1] - h8_inner[1]), self.board_z_max]
        # x_vector_2 = [(a1_inner[0] - h1_inner[0]), (a1_inner[1] - h1_inner[1]), self.board_z_max]
        # x_vector = [sum(x)/2 for x in zip(x_vector_1, x_vector_2)]
        # self.x_unit_vector = [coord/6 for coord in x_vector]

        # Find y axis vector
        y_vector_1 = [(h1[0] - h8[0]), (h1[1] - h8[1]), self.board_z_max]
        y_vector_2 = [(a1[0] - a8[0]), (a1[1] - a8[1]), self.board_z_max]
        y_vector = [sum(x) / 2 for x in zip(y_vector_1, y_vector_2)]
        self.y_unit_vector = [coord / 8 for coord in y_vector]

        # y_vector_1 = [(h1_inner[0] - h8_inner[0]), (h1_inner[1] - h8_inner[1]), self.board_z_max]
        # y_vector_2 = [(a1_inner[0] - a8_inner[0]), (a1_inner[1] - a8_inner[1]), self.board_z_max]
        # y_vector = [sum(x)/2 for x in zip(y_vector_1, y_vector_2)]
        # self.y_unit_vector = [coord/6 for coord in y_vector]

        # self.H8 = h8_inner - x_unit_vector - y_unit_vector

        # Find hover height
        self.hover_height = self.board_z_max + 0.2  # hard coded hover height

        # Find location of dead zone
        dead_zone_x_vector = [(i * 12) for i in self.x_unit_vector]
        dead_zone_y_vector = [i * 4 for i in self.y_unit_vector]
        dead_zone_z = self.board_z_max + 0.15  # hardcoded z coordinate of dead zone
        dead_zone = [sum(i) for i in zip(self.H8, dead_zone_x_vector, dead_zone_y_vector)]
        dead_zone[2] = dead_zone_z
        self.dead_zone = dead_zone

        # Find the location of the rest position
        rest_x_vector = [i * 4 for i in self.x_unit_vector]
        rest_y_vector = [(i * 4) for i in self.y_unit_vector]
        rest_z = self.board_z_max + 0.4  # hardcoded z coordinate of the rest position
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

        # Find the correct displacement for each piece
        offset = 0.05
        self.piece_dims = dict([('p', [(0.061 - offset), 0.132]),
                                ('k', [(0.064 - offset), 0.130]),
                                ('q', [(0.065 - offset), 0.138]),
                                ('b', [(0.066 - offset), 0.134]),
                                ('n', [(0.072 - offset), 0.096]),
                                ('r', [(0.063 - offset), 0.120])
                                ])

    def generate_chess_motion(self, chess_move):
        """Called after chess move has been determined

        # Chess move is formatted as either of the below
        # chess_move = [("r", "a1a6")]
        # chess_move = [("r", "a6"), ("r", "a1a6")]
        """
        # Extract information from output of game engine
        if len(chess_move) == 1:
            start_an = chess_move[0][1][:2]
            goal_an = chess_move[0][1][2:4]
        elif len(chess_move) == 2:
            start_an = chess_move[1][1][:2]
            goal_an = chess_move[1][1][2:4]
        else:
            raise ValueError("Chess move was not understood; not 1 or 2 items")

        print("start an", start_an)
        print("goal_an", goal_an)

        # Convert ANs to coordinates
        move_from = self.an_to_coordinates(start_an)
        move_to = self.an_to_coordinates(goal_an)

        # Generate the intermediate positions of the path
        move_from_hover = [move_from[0], move_from[1], self.hover_height]
        move_to_hover = [move_to[0], move_to[1], self.hover_height]
        dead_zone_hover = [self.dead_zone[0], self.dead_zone[1], self.hover_height]

        if len(chess_move) == 1:  # NO PIECE DIED
            path = [self.rest, move_from_hover, move_from, move_from_hover, move_to_hover,
                    move_to, move_to_hover, self.rest]

            # TAKE CARE WHEN DESIGNING THESE PLAYS, THERE CANNOT BE TWO CONSECUTIVE IDENTICAL POINTS
            play = [[self.rest, move_from_hover, move_from],
                    [move_from, move_from_hover, move_to_hover, move_to],
                    [move_to, move_to_hover, self.rest]]

        elif len(chess_move) == 2:  # A PIECE HAS DIED
            path = [self.rest, move_to_hover, move_to, move_to_hover, dead_zone_hover,
                    dead_zone_hover, move_from_hover, move_from, move_from_hover,
                    move_to_hover, move_to, move_to_hover, self.rest]

            # TAKE CARE WHEN DESIGNING THESE PLAYS, THERE CANNOT BE TWO CONSECUTIVE IDENTICAL POINTS
            play = [[self.rest, move_to_hover, move_to],
                    [move_to, move_to_hover, dead_zone_hover],
                    [dead_zone_hover, move_from_hover, move_from],
                    [move_from, move_from_hover, move_to_hover, move_to],
                    [move_to, move_to_hover, self.rest]]
        else:
            raise ValueError("The length of chess move tuple is invalid")

        # for each move in play list we want to discretise it and remove any overlapping points
        # between segments
        for index, move in enumerate(play):
            move_discrete = self.discretise_path(move, self.dx)
            play[index] = move_discrete  # overwrite in the play list

        # smooth the corners using chaser-leader
        smooth_paths = []
        for move in play:
            smooth_paths.append(self.smooth_corners(move, size_of_corner=10, passes=6))  # 12, 5

        # TODO add velocity profiling and sampling

        if self.visual:
            # Separate into xyz
            x = [coord[0] for coord in path]
            y = [coord[1] for coord in path]
            z = [coord[2] for coord in path]

            # getting board points
            board_x = [coord[0] for coord in self.board]
            board_y = [coord[1] for coord in self.board]
            board_z = [coord[2] for coord in self.board]
            # add the first point to the end of the list to create polygon
            board_x.append(self.board[0][0])
            board_y.append(self.board[0][1])
            board_z.append(self.board[0][2])

            # plotting
            fig = plt.figure()
            ax3d = fig.add_subplot(111, projection='3d')

            # plot the board
            ax3d.plot(board_x, board_y, board_z, 'y')
            # plot important points
            ax3d.plot([self.rest[0]], [self.rest[1]], [self.rest[2]], 'g*')
            ax3d.plot([self.dead_zone[0]], [self.dead_zone[1]], [self.dead_zone[2]], 'g*')
            # plot the untouched path
            ax3d.plot(x, y, z, 'r')
            # plot the smooth paths
            for i in range(len(smooth_paths)):
                ax3d.plot(smooth_paths[i][:, 0], smooth_paths[i][:, 1], smooth_paths[i][:, 2], 'b*')

            plt.show()

    def an_to_coordinates(self, an):  # todo fix this function for kill move
        """Converts algebraic notation into real world ``x, y, z`` coordinates"""
        # split AN location
        letter = an[0]
        number = an[1]

        # lookup in dictionary
        x_in_chess = self.number_dict[number]
        y_in_chess = self.letter_dict[letter]

        # move by dims_in_chess from the H8 coordinate
        coord = [sum(i) for i in zip(self.H8, x_in_chess, y_in_chess)]
        # overwrite z coordinate with predetermined z height
        coord[2] = self.board_z_max
        return coord

    @staticmethod
    def discretise(point_1, point_2, dx):  # todo clean up function
        """Generates a list of coordinates ``n`` metres apart between 2 points ``a`` and ``b`` in
        space."""

        # create vector from point_1 to point_2
        vector = [point_2[0] - point_1[0], point_2[1] - point_1[1], point_2[2] - point_1[2]]
        # noinspection PyUnresolvedReferences
        distance = np.sqrt(sum(i ** 2 for i in vector))

        # number of points on line
        i = int(distance / dx)

        # discretise by creating new 1d array
        line_x = np.linspace(point_1[0], point_2[0], i)
        line_y = np.linspace(point_1[1], point_2[1], i)
        line_z = np.linspace(point_1[2], point_2[2], i)
        # noinspection PyUnresolvedReferences
        line = np.array(np.transpose(np.vstack((line_x, line_y, line_z))))
        return line

    def discretise_path(self, move, dx):
        """
        Discretise a moves path using object defined dx for unit.
        :param move: list of points path goes through
        :param dx: displacement between two points on the target discretised path
        :return: discretised path
        """
        move_discrete = []
        # iterate through move segments, discretise and join them
        for seg_idx in range(len(move) - 1):
            current_segment = self.discretise(move[seg_idx], move[seg_idx + 1], dx)

            # print(current_segment)
            # we add our discretised segment to our move
            if seg_idx > 0:
                # if the end of our current move is the same position as the start of our new
                # segment then we only want to add the list from the second point onwards
                if move_discrete[-1][0] == current_segment[0][0]:
                    # noinspection PyUnresolvedReferences
                    move_discrete = np.concatenate((move_discrete, current_segment[1:]))
                else:
                    # noinspection PyUnresolvedReferences
                    move_discrete = np.concatenate((move_discrete, current_segment))

            else:  # on first iteration, we store our segment directly
                move_discrete = current_segment
        return move_discrete

    @staticmethod
    def smooth_corners(path, size_of_corner, passes):  # TODO: FIX this function
        """Takes a discretised path and and rounds the corners using parameters passed into
        function call. Minimum number of passes is 1, which results in a chamfer."""
        if not size_of_corner % 2 == 0:  # number of steps must be an even number
            size_of_corner += 1
        steps = size_of_corner
        if passes < 1:
            raise ValueError("Number of passes must be >= 1")

        for i in range(passes):
            trajectory_1_x = [item[0] for item in path]
            trajectory_1_y = [item[1] for item in path]
            trajectory_1_z = [item[2] for item in path]

            x_tortoise = trajectory_1_x[:-steps]  # remove last few coords
            x_hare = trajectory_1_x[steps:]  # first last few coords
            x_smooth_path = [(sum(i) / 2) for i in zip(x_tortoise, x_hare)]  # average them

            y_tortoise = trajectory_1_y[:-steps]  # remove last few coords
            y_hare = trajectory_1_y[steps:]  # remove first few coords
            y_smooth_path = [(sum(i) / 2) for i in zip(y_tortoise, y_hare)]

            z_tortoise = trajectory_1_z[:-steps]  # remove last few coords
            z_hare = trajectory_1_z[steps:]  # remove first few coords
            z_smooth_path = [(sum(i) / 2) for i in zip(z_tortoise, z_hare)]

            # noinspection PyUnresolvedReferences
            smooth_path = np.array(
                [list(i) for i in zip(x_smooth_path, y_smooth_path, z_smooth_path)])
            # append first 6 coords in trajectory_1 to the smooth_path

            # print(path[:(steps / 2)])
            # print(smooth_path)

            # noinspection PyUnresolvedReferences
            smooth_path = np.concatenate(
                (path[:(steps / 2)], smooth_path, path[-(steps / 2):]), axis=0)
            path = smooth_path
        return path

    @staticmethod
    def length_of_path(path):  # todo docstring
        """Takes the a path array of (n x 3) and returns the length of path."""
        length = 0
        for i in range(len(path) - 1):
            point_a = path[i]
            point_b = path[i + 1]
            length += np.sqrt((point_b[0] - point_a[0]) ** 2 + (point_b[1] - point_a[1]) ** 2
                              + (point_b[2] - point_a[2]) ** 2)
        return length

    def apply_trapezoid_vel_profile(self, path):  # todo docstring and finish the function
        # set rate of message sending:  0.001 sec == dt == 1kHz  NOTE THIS IS GLOBALLY SET
        dt = 0.005
        # set acceleration, start with 0.1 (may need to reduce)  NOTE THIS IS GLOBALLY SET
        acc = 0.08  # max 1.0
        # set target travel speed for motion
        target_speed = 0.1  # max 1.0

        # discretise using a max unit of:  targV * dt # TODO link this to class
        # this unit should be 1/(acc * dt**2) to ensure there is one distance sample
        # dx = target_speed * dt  # this is the maximum delta displacement
        dx = acc * dt ** 2  # this is the ideal delta value
        if self.debug:
            print("delta displacement (mm): ", dx * 1000)

        dis_path = planner.discretise_path(path, dx)

        # TODO: if smoothing is going to happen it MUST keep consistent delta displacement
        # run chaser/leader over path to generate a new one with same resolution
        # corner = 0.05  # in meters
        # steps = int(corner * 2 / dx)
        # print("Steps: ", steps)
        # smooth_path = planner.smooth_corners(dis_path, size_of_corner=steps, passes=6)
        smooth_path = dis_path

        # find the length of the new path
        lop = self.length_of_path(smooth_path)
        if self.debug:
            print("LOP: ", lop)

        # check if the length of path is < ( speed**2/acc )
        minimum_path_length = target_speed ** 2 / acc
        if self.debug:
            print("Minimum path length: ", minimum_path_length)
        if lop < minimum_path_length:
            # if the length is less we need to reduce target_speed
            if self.debug:
                print("Path length is too short.")
            old_speed = target_speed
            target_speed = np.sqrt(lop * acc)
            if self.debug:
                print("Target speed changed from: ", old_speed, ", to: ", target_speed)

            # assert new target_speed is less than old for safety reasons
            assert (target_speed <= old_speed)

        else:
            # we have confirmed the length of the path is long enough for our target speed
            if self.debug:
                print("Path length ok")

        # we now need to create the speed profile graph and define its parameters

        # find t for acceleration and deceleration
        end_stage_t = target_speed / acc
        # find path distance for acc and dec
        end_stage_displacement = end_stage_t * target_speed / 2

        # find displacement for constant speed section of motion
        mid_stage_displacement = lop - 2 * end_stage_displacement
        # find t for const speed section
        mid_stage_t = mid_stage_displacement / target_speed

        # find total time
        total_time = end_stage_t * 2 + mid_stage_t
        if self.debug:
            print("total time: ", total_time)

        # create a time list using 0->T in steps of dt
        time_list = np.arange(start=0, stop=total_time, step=dt)
        np.reshape(time_list, (np.shape(time_list)[0], 1))

        # sample speed graph to create list to go with time list
        speed_values = []
        c = (0 - (-acc) * time_list[-1])
        for t in time_list:
            if t <= end_stage_t:
                # acceleration period
                speed_values.append(acc * t)

            elif t >= end_stage_t + mid_stage_t:
                # deceleration stage
                speed_values.append(-acc * t + c)

            elif t > end_stage_t:
                # constant speed at target speed
                speed_values.append(target_speed)

        # sample path using speed list
        trajectory = np.hstack((smooth_path[0, :], speed_values[0]))  # send intermediate points
        # trajectory = np.hstack((smooth_path[0, :], 0.1))
        # trajectory = np.hstack((path[-1, :], speed_values[0]))  # only send the goal position
        smooth_path_idx = 0

        for i in range(1, len(speed_values)):
            samples = int(np.rint(speed_values[i] * dt / dx))
            smooth_path_idx += samples
            if smooth_path_idx > len(smooth_path) - 1:
                smooth_path_idx = len(smooth_path) - 1
            new_marker = np.hstack(
                (smooth_path[smooth_path_idx], speed_values[i]))  # send intermediate points
            # new_marker = np.hstack((path[-1, :], speed_values[i]*0.97))  # only send the goal position

            # new_marker = np.hstack((smooth_path[smooth_path_idx], 0.1))
            trajectory = np.vstack((trajectory, new_marker))

        # for i in range(3):
        #     new_marker = np.hstack((smooth_path[-1], speed_values[-1]))
        #     trajectory = np.vstack((trajectory, new_marker))

        # trajectory[-1, -1] = 0

        if self.visual:
            # plotting the board
            # plot discretised path
            # fig = plt.figure()
            # ax3d = fig.add_subplot(111, projection='3d')
            # ax3d.plot(path[:, 0], path[:, 1], path[:, 2], 'r')
            # ax3d.plot(smooth_path[:, 0], smooth_path[:, 1], smooth_path[:, 2], 'b*')

            # plot speed profile
            fig = plt.figure()
            ax3d = fig.add_subplot(111)
            plt.plot(time_list[:len(speed_values)], speed_values, 'r*')
            plt.ylabel("speed (m/s)")
            plt.xlabel("time (s)")

            # plot trajectory axes against time
            fig = plt.figure()
            ax3d = fig.add_subplot(111)
            plt.plot(time_list[:len(trajectory[:, 0])], trajectory[:, 0], 'r*')
            plt.plot(time_list[:len(trajectory[:, 1])], trajectory[:, 1], 'b*')
            plt.plot(time_list[:len(trajectory[:, 2])], trajectory[:, 2], 'g*')
            plt.ylabel("x(r) / y(b) / z(g) displacement (m)")
            plt.xlabel("time (s)")

            # plot trajectory in 3d
            # fig = plt.figure()
            # ax3d = fig.add_subplot(111, projection='3d')
            # ax3d.plot(path[:, 0], path[:, 1], path[:, 2], 'r')
            # ax3d.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 'g*')

            plt.show()

        return trajectory

    def anna(self, chess_move, franka):
        """Creates a list of lists that depict the full start to goal trajectory [start to hover][hover to etc]"""

        # Extract information from output of game engine
        if len(chess_move) == 1:
            start_an = chess_move[0][1][:2]
            goal_an = chess_move[0][1][2:4]
        elif len(chess_move) == 2:
            start_an = chess_move[1][1][:2]
            goal_an = chess_move[1][1][2:4]
        else:
            raise ValueError("Chess move was not understood; not 1 or 2 items")

        # Convert ANs to coordinates
        move_from = self.an_to_coordinates(start_an)
        move_to = self.an_to_coordinates(goal_an)

        # Generate the intermediate positions of the path
        move_from_hover = [move_from[0], move_from[1], self.hover_height]
        move_to_hover = [move_to[0], move_to[1], self.hover_height]
        dead_zone_hover = [self.dead_zone[0], self.dead_zone[1], self.hover_height]

        # current_position = # TODO get position
        current_position = [1, 1, 1]
        current_position = [franka.x, franka.y, franka.z]

        # create numpy array of tuples to move from and to
        if len(chess_move) == 1:  # NO PIECE DIED

            moves = np.array([[[current_position, self.rest], [current_position, move_from_hover],
                               [current_position, move_from]],
                              [[current_position, move_from_hover],
                               [current_position, move_to_hover], [current_position, move_to]],
                              [[current_position, move_to_hover], [current_position, self.rest]]])

        elif len(chess_move) == 2:  # A PIECE HAS DIED

            moves = np.array([[[current_position, self.rest], [current_position, move_to_hover],
                               [current_position, move_to]],
                              [[current_position, move_to_hover],
                               [current_position, dead_zone_hover]],
                              [[current_position, move_from_hover], [current_position, move_from]],
                              [[current_position, move_from_hover],
                               [current_position, move_to_hover], [current_position, move_to]],
                              [[current_position, move_to_hover], [current_position, self.rest]]])

        else:
            raise ValueError("The length of chess move tuple is invalid")

        # # plot the trajectory
        # if self.visual:
        #     fig = plt.figure()
        #     ax3d = fig.add_subplot(111, projection='3d')

        return moves


if __name__ == '__main__':
    arm = None
    from franka.franka_control_ros import FrankaRos

    arm = FrankaRos(log=True)

    # print("entering true loop")
    # while True:
    #     while arm.z is None:
    #         print("waiting for chage in z")
    #         time.sleep(1)
    #     print("x: ", arm.x)
    #     print("y: ", arm.y)
    #     print("z: ", arm.z)
    #     time.sleep(1)

    # rospy.Subscriber("franka_current_position", Float64MultiArray, cur_pos_callback, queue_size=1)
    # spinthread = thread.start_new_thread( spinner , ())
    # print("Waiting for current position")
    # while glob_curr_pos_z is None:
    #     time.sleep(0.1)
    # print("current position changed")

    planner = MotionPlanner(arm, visual=False, manual_calibration=False, debug=True)
    # x, y, z = arm.get_position()
    # path_4 = np.array([[x, y, z], [x, y+0.5, z]])  # move +y

    # planner.generate_chess_motion([("r", "a1a6")])  # example of simple move
    # TODO fix for dead piece generation
    # FRANKA.generate_chess_motion([("r", "b4"), ("r", "a1b4")])  # example of move with kill

    # path_1 = np.array([[0, 0, 0], [0, 1, 0], [1, 1, 0]])  # right angle turn
    # path_2 = np.array([[1, 2, 0], [2, 0.5, 0], [4, 4, 0]])  # tight angle turn
    # path_3 = np.array([[0, 0, 0], [0, 0.5, 0]])  # straight line example
    # path_4 = np.array([[0.5, -0.25, 0.2], [0.5, 0.25, 0.2]])  # move +y

    path_5 = np.array([[arm.x, arm.y, arm.z], [arm.x, arm.y, arm.z - 0.2]])  # move +y

    # motion_plan = planner.apply_trapezoid_vel_profile(path_5)

    ## ANNA TEST STUFF
    # 

    ungrip_dim = 0.045
    time.sleep(1)
    arm.move_gripper(ungrip_dim, 0.1)
    time.sleep(1)

    if True:
        import rospy

        try:
            # print(motion_plan)
            # for x, y, z, speed in motion_plan:
            #     # print(x,y,z,speed)
            #     arm.move_to(x, y, z, speed)
            #     time.sleep(0.005)  # control loop

            # chess_move = [("r", "a1a6")]
            # chess_move = [("r", "h8"), ("p", "a1h8")]
            chess_move = [("r", "h6"), ("r", "h1h6")]
            moves = planner.anna(chess_move, arm)
            current_position = [arm.x, arm.y, arm.z]

            for i, series in enumerate(moves):
                series = np.array(series)

                for path in series:
                    path = np.array(path)
                    path[0] = current_position
                    # # plot 2 points
                    # if self.visual:
                    #     ax3d.plot(path[:, 0], path[:, 1], path[:, 2], 'r*')
                    #     ax3d.plot(path[:, 0], path[:, 1], path[:, 2], 'r')

                    motion_plan = planner.apply_trapezoid_vel_profile(path)

                    arm.send_trajectory(motion_plan)
                    # for x, y, z, speed in motion_plan:
                    #     # print(x,y,z,speed)
                    #     arm.move_to(x, y, z, speed)
                    #     time.sleep(0.005)  # control loop

                    # update current position
                    current_position = [arm.x, arm.y, arm.z]

                # gripper starts here

                if len(chess_move) == 1:  # if not piece died
                    piece = chess_move[0][0].lower()  # which is the piece?
                    dims = planner.piece_dims[piece]  # height and grip dims
                    grip_dim = dims[0]  # grip dims
                    if i == 0:
                        # grip
                        time.sleep(1)
                        arm.move_gripper(grip_dim, 0.1)
                        time.sleep(1)
                    elif i == 1:
                        # release
                        time.sleep(1)
                        arm.move_gripper(ungrip_dim, 0.1)
                        time.sleep(1)

                elif len(chess_move) == 2:
                    piece = chess_move[0][0].lower()
                    dims = planner.piece_dims[piece]
                    grip_dim = dims[0]

                    # PICKING UP DEAD PIECE
                    if i == 0:
                        # grip
                        time.sleep(1)
                        arm.move_gripper(grip_dim, 0.1)
                        time.sleep(1)

                    # DROPPING OFF DEAD PIECE AT DEAD ZONE
                    elif i == 1:
                        # release
                        time.sleep(1)
                        arm.move_gripper(ungrip_dim, 0.1)
                        time.sleep(1)

                    # PICKING UP NEW PIECE
                    elif i == 2:
                        piece = chess_move[1][0]  # change piece dims for second grip
                        dims = planner.piece_dims[piece]
                        grip_dim = dims[0]

                        # grip
                        time.sleep(1)
                        arm.move_gripper(grip_dim, 0.1)
                        time.sleep(1)

                    # DROPPING OFF NEW PIECE TO ITS DESTINATION
                    elif i == 3:
                        # release
                        time.sleep(1)
                        arm.move_gripper(ungrip_dim, 0.1)
                        time.sleep(1)

            time.sleep(2)
            print("finished motion.")
            print("final x: ", arm.x)
            print("final y: ", arm.y)
            print("final z: ", arm.z)
            print("offset x (mm): ", (arm.x - motion_plan[-1, 0]) / 1000)
            print("offset y (mm): ", (arm.y - motion_plan[-1, 1]) / 1000)
            print("offset z (mm): ", (arm.z - motion_plan[-1, 2]) / 1000)

        except rospy.ROSInterruptException as e:
            print(e)
