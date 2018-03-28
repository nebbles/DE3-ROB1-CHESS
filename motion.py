from __future__ import print_function
import numpy as np
import time
# noinspection PyUnresolvedReferences
from std_msgs.msg import Float64MultiArray
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
# noinspection PyUnresolvedReferences
from mpl_toolkits.mplot3d import Axes3D  # for: fig.gca(projection = '3d')

glob_curr_pos_x = None
glob_curr_pos_y = None
glob_curr_pos_z = None

try:  # Python 2/3 raw input correction
    # noinspection PyShadowingBuiltins
    input = raw_input  # overwrite the use of input with raw_input to support Python 2
except NameError:
    pass


class MotionPlanner:
    def __init__(self, visual=False, debug=False):
        """
        Initialises the MotionPlanner class with preset hard-coded calibration unless the
        automatic calibration procedure is implemented.

        :param visual: Flag control usage of matplotlib for checking results.
        :param debug: Flag for controlling extensive print statements to console.
        """

        self.visual = visual
        self.dx = 0.005
        self.debug = debug

        a1_inner = [0.7267045425008777, -0.19296634171907032, -0.06872549226985156]
        a8_inner = [0.3927532235233764, -0.18683619713296443, -0.07657893758846493]
        h8_inner = [0.4011762302881528, 0.1679801463159881, -0.07535828159982629]
        h1_inner = [0.7364192616073851, 0.1560582695967688, -0.06846008006252183]

        self.board = [a1_inner, a8_inner, h8_inner, h1_inner]
        # Find the maximum z value of the board
        self.board_z_max = -0.075

        # from Paolo's collected coords
        x_vector_1 = [(a8_inner[0] - h8_inner[0]), (a8_inner[1] - h8_inner[1]), self.board_z_max]
        x_vector_2 = [(a1_inner[0] - h1_inner[0]), (a1_inner[1] - h1_inner[1]), self.board_z_max]
        x_vector = [sum(x)/2 for x in zip(x_vector_1, x_vector_2)]
        self.x_unit_vector = [coord/6 for coord in x_vector]

        y_vector_1 = [(h1_inner[0] - h8_inner[0]), (h1_inner[1] - h8_inner[1]), self.board_z_max]
        y_vector_2 = [(a1_inner[0] - a8_inner[0]), (a1_inner[1] - a8_inner[1]), self.board_z_max]
        y_vector = [sum(x)/2 for x in zip(y_vector_1, y_vector_2)]
        self.y_unit_vector = [coord/6 for coord in y_vector]

        print(self.x_unit_vector)
        print(self.y_unit_vector)
        print(h8_inner)

        self.H8 = [A-B-C for A, B, C in zip(h8_inner, self.x_unit_vector, self.y_unit_vector)]

        # self.H8 = h8_inner - self.x_unit_vector - self.y_unit_vector
        print(self.H8)

        # Find hover height
        self.hover_height = self.board_z_max + 0.2  # hard coded hover height

        # Find location of dead zone
        dead_zone_x_vector = [(vector * 12) for vector in self.x_unit_vector]
        dead_zone_y_vector = [vector * 4 for vector in self.y_unit_vector]
        dead_zone_z = self.board_z_max + 0.15  # hardcoded z coordinate of dead zone
        dead_zone = [sum(elements) for elements in zip(self.H8, dead_zone_x_vector,
                                                       dead_zone_y_vector)]
        dead_zone[2] = dead_zone_z
        self.dead_zone = dead_zone

        # Find the location of the rest position
        rest_x_vector = [vector * 4 for vector in self.x_unit_vector]
        rest_y_vector = [(vector * 4) for vector in self.y_unit_vector]
        rest_z = self.board_z_max + 0.5  # hardcoded z coordinate of the rest position
        rest = [sum(element) for element in zip(self.H8, rest_x_vector, rest_y_vector)]
        rest[2] = rest_z
        self.rest = rest

        # Find coordinates of each square
        self.letter_dict = dict([('h', [vector * 0.5 for vector in self.x_unit_vector]),
                                 ('g', [vector * 1.5 for vector in self.x_unit_vector]),
                                 ('f', [vector * 2.5 for vector in self.x_unit_vector]),
                                 ('e', [vector * 3.5 for vector in self.x_unit_vector]),
                                 ('d', [vector * 4.5 for vector in self.x_unit_vector]),
                                 ('c', [vector * 5.5 for vector in self.x_unit_vector]),
                                 ('b', [vector * 6.5 for vector in self.x_unit_vector]),
                                 ('a', [vector * 7.5 for vector in self.x_unit_vector])
                                 ])

        self.number_dict = dict([('8', [vector * 0.5 for vector in self.y_unit_vector]),
                                 ('7', [vector * 1.5 for vector in self.y_unit_vector]),
                                 ('6', [vector * 2.5 for vector in self.y_unit_vector]),
                                 ('5', [vector * 3.5 for vector in self.y_unit_vector]),
                                 ('4', [vector * 4.5 for vector in self.y_unit_vector]),
                                 ('3', [vector * 5.5 for vector in self.y_unit_vector]),
                                 ('2', [vector * 6.5 for vector in self.y_unit_vector]),
                                 ('1', [vector * 7.5 for vector in self.y_unit_vector])
                                 ])

        # Find the correct displacement for each piece
        # Dictionary holds the required gripper width and z offset from table for each piece type
        width_offset = 0.05
        z_offset = self.board_z_max
        self.piece_dims = dict([('p', [(0.061 - width_offset), 0.030+z_offset]),
                                ('k', [(0.064 - width_offset), 0.063+z_offset]),
                                ('q', [(0.065 - width_offset), 0.063+z_offset]),
                                ('b', [(0.066 - width_offset), 0.049+z_offset]),
                                ('n', [(0.060 - width_offset), 0.023+z_offset]),
                                ('r', [(0.063 - width_offset), 0.029+z_offset])
                                ])

    @staticmethod
    def discretise(point_1, point_2, dx):
        """
        Takes a straight line and divides it into smaller defined length segments.

        :param point_1: First point in 3D space
        :param point_2: Second point in 3D space
        :param dx: Distance between points in discretised line.
        :return: Numpy array of discretised line.
        """
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
        line = np.array(np.transpose(np.vstack((line_x, line_y, line_z))))
        return line

    def discretise_path(self, move, dx):
        """
        Discretise a moves path using object defined dx for unit.

        :param move: List of points path goes through.
        :param dx: Displacement between two points on the target discretised path.
        :return: Discretised path as numpy array.
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
    def smooth_corners(path, size_of_corner, passes):
        """
        Takes a discretised path and and rounds the corners using parameters passed into
        function call. Minimum number of passes is 1, which results in a chamfer.

        **Note** This function is not currently used due to its error in smoothing the corners.
        It has not been implemented until a better version can be written.
        """
        if not size_of_corner % 2 == 0:  # number of steps must be an even number
            size_of_corner += 1
        steps = size_of_corner
        if passes < 1:
            raise ValueError("Number of passes must be >= 1")

        for i in range(passes):
            trajectory_1_x = [item[0] for item in path]
            trajectory_1_y = [item[1] for item in path]
            trajectory_1_z = [item[2] for item in path]

            x_tortoise = trajectory_1_x[:-steps]  # remove last few coordinates
            x_hare = trajectory_1_x[steps:]  # first last few coordinates
            x_smooth_path = [(sum(i) / 2) for i in zip(x_tortoise, x_hare)]  # average them

            y_tortoise = trajectory_1_y[:-steps]  # remove last few coordinates
            y_hare = trajectory_1_y[steps:]  # remove first few coordinates
            y_smooth_path = [(sum(i) / 2) for i in zip(y_tortoise, y_hare)]

            z_tortoise = trajectory_1_z[:-steps]  # remove last few coordinates
            z_hare = trajectory_1_z[steps:]  # remove first few coordinates
            z_smooth_path = [(sum(i) / 2) for i in zip(z_tortoise, z_hare)]

            # noinspection PyUnresolvedReferences
            smooth_path = np.array(
                [list(i) for i in zip(x_smooth_path, y_smooth_path, z_smooth_path)])
            # append first 6 coordinates in trajectory_1 to the smooth_path

            # print(path[:(steps / 2)])
            # print(smooth_path)

            # noinspection PyUnresolvedReferences
            smooth_path = np.concatenate(
                (path[:(steps / 2)], smooth_path, path[-(steps / 2):]), axis=0)
            path = smooth_path
        return path

    @staticmethod
    def length_of_path(path):
        """
        Calculates the length of a path stored as an array of (n x 3).

        :param path: List (length n) of list (length 3) points.
        :return: The total length of the path in 3D space.
        """
        length = 0
        for i in range(len(path) - 1):
            point_a = path[i]
            point_b = path[i + 1]
            length += np.sqrt((point_b[0] - point_a[0]) ** 2 + (point_b[1] - point_a[1]) ** 2
                              + (point_b[2] - point_a[2]) ** 2)
        return length

    def apply_trapezoid_vel(self, path, acceleration=0.02, max_speed=0.8):
        """
        Takes a path (currently only a start/end point (straight line), and returns a discretised
        trajectory of the path controlled by a trapezium velocity profile generated by the input
        parameters.

        :param path: List of two points in 3D space.
        :param acceleration: Acceleration and deceleration of trapezium profile.
        :param max_speed: Target maximum speed of the trapezium profile.
        :return: Trajectory as numpy array.
        """
        # set rate of message sending:  0.001 sec == dt == 1kHz  NOTE THIS IS GLOBALLY SET
        dt = 0.005
        # set acceleration, start with 0.1 (may need to reduce)  NOTE THIS IS GLOBALLY SET
        acc = acceleration  # max 1.0
        # set target travel speed for motion
        target_speed = max_speed  # max 1.0

        # discretise using a max unit of:  target_speed * dt
        # ideally, we use a dx of: acc * dt**2
        dx = acc * dt**2  # this is the ideal delta value

        if self.debug:
            print("delta displacement (mm): ", dx * 1000)

        lop = 0
        while lop == 0:
            dis_path = self.discretise_path(path, dx)

            # SMOOTHING HAPPENS HERE
            # if smoothing is going to happen it MUST keep consistent delta displacement
            # corner = 0.05  # in meters
            # steps = int(corner * 2 / dx)
            # print("Steps: ", steps)
            # smooth_path = planner.smooth_corners(dis_path, size_of_corner=steps, passes=6)
            smooth_path = dis_path  # disable smoothing

            # find the length of the new path
            lop = self.length_of_path(smooth_path)
            if self.debug:
                print("LOP: ", lop)
            if lop == 0:
                print("Length of path is zero, adding indistinguishable offset.")
                path[1] = [c+0.000001 for c in path[1]]

        # check if the length of path is < ( speed**2/acc )
        # this means that the length of the path is too short to accelerate all the way to the
        # target speed so we scale it down to keep a triangular profile
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
        print("Acc/dec time: ", end_stage_t)

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
        # noinspection PyUnboundLocalVariable
        trajectory = np.hstack((smooth_path[0, :], speed_values[0]))  # send intermediate points
        smooth_path_idx = 0

        for i in range(1, len(speed_values)):
            samples = int(np.rint(speed_values[i] * dt / dx))
            smooth_path_idx += samples
            if smooth_path_idx > len(smooth_path) - 1:
                smooth_path_idx = len(smooth_path) - 1
            new_marker = np.hstack((smooth_path[smooth_path_idx], speed_values[i]))
            trajectory = np.vstack((trajectory, new_marker))

        if self.visual:
            # plotting the board
            # plot discretised path
            # fig = plt.figure()
            # ax3d = fig.add_subplot(111, projection='3d')
            # ax3d.plot(path[:, 0], path[:, 1], path[:, 2], 'r')
            # ax3d.plot(smooth_path[:, 0], smooth_path[:, 1], smooth_path[:, 2], 'b*')

            # plot speed profile
            fig = plt.figure()
            # noinspection PyUnusedLocal
            ax3d = fig.add_subplot(111)
            plt.plot(time_list[:len(speed_values)], speed_values, 'r*')
            plt.ylabel("speed (m/s)")
            plt.xlabel("time (s)")

            # plot trajectory axes against time
            fig = plt.figure()
            # noinspection PyUnusedLocal
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

    def an_to_coordinates(self, chess_move_an):
        """
        Converts chess algebraic notation into real world ``x, y, z`` coordinates.

        :param chess_move_an: In form ``[('p','a2a4')]`` or ``[('n','a4'),('p','a2a4')]``.
        :return: Tuple of coordinate start(1), goal(2), died(3).
        """

        # Extract information from output of game engine
        if len(chess_move_an) == 1:
            start_an = chess_move_an[0][1][:2]
            goal_an = chess_move_an[0][1][2:4]
        elif len(chess_move_an) == 2:
            start_an = chess_move_an[1][1][:2]
            goal_an = chess_move_an[1][1][2:4]
        else:
            raise ValueError("Chess move was not understood; not 1 or 2 items")

        # split start AN location
        letter_start = start_an[0]
        number_start = start_an[1]

        # split goal AN location
        letter_goal = goal_an[0]
        number_goal = goal_an[1]

        # lookup in dictionary
        x_in_chess_start = self.number_dict[number_start]
        y_in_chess_start = self.letter_dict[letter_start]

        x_in_chess_goal = self.number_dict[number_goal]
        y_in_chess_goal = self.letter_dict[letter_goal]

        # move by dims_in_chess from the H8 coordinate
        coord_start = [sum(i) for i in zip(self.H8, x_in_chess_start, y_in_chess_start)]
        coord_goal = [sum(i) for i in zip(self.H8, x_in_chess_goal, y_in_chess_goal)]
        coord_died = [sum(i) for i in zip(self.H8, x_in_chess_goal, y_in_chess_goal)]

        # select z coordinate based on what piece it is
        if len(chess_move_an) == 1:
            piece = chess_move_an[0][0].lower()  # which is the piece?
            dims = self.piece_dims[piece]  # height and grip dims
            coord_start[2] = dims[1]  # height dim
            coord_goal[2] = dims[1]  # height dim
            return coord_start, coord_goal

        elif len(chess_move_an) == 2:
            alive_piece = chess_move_an[1][0].lower()  # which is the piece?
            dead_piece = chess_move_an[0][0].lower()

            alive_dims = self.piece_dims[alive_piece]  # height and grip dims
            dead_dims = self.piece_dims[dead_piece]  # height and grip dims

            coord_start[2] = alive_dims[1]  # height dim
            coord_goal[2] = alive_dims[1]  # height dim
            coord_died[2] = dead_dims[1]

            return coord_start, coord_goal, coord_died

    def generate_moves(self, chess_move_an, franka):
        """
        Generates a number of segments each a straight line path that will execute the move
        generated by the chess engine.

        :param chess_move_an: Takes chess move in algebraic notation from chess engine
        :param franka: Takes franka control object as argument to find current position
        :return: a list of lists that depict the full start to goal trajectory of each segment
        """
        if len(chess_move_an) == 1:
            move_from, move_to = self.an_to_coordinates(chess_move_an)

            # Generate the intermediate positions of the path
            move_from_hover = [move_from[0], move_from[1], self.hover_height]
            move_to_hover = [move_to[0], move_to[1], self.hover_height]

            current_position = [franka.x, franka.y, franka.z]

            moves = np.array([[[current_position, self.rest], [current_position,  move_from_hover],
                               [current_position, move_from]],
                              [[current_position, move_from_hover],
                               [current_position, move_to_hover], [current_position, move_to]],
                              [[current_position, move_to_hover], [current_position, self.rest]]])

        elif len(chess_move_an) == 2:
            move_from, move_to, died = self.an_to_coordinates(chess_move_an)

            # Generate the intermediate positions of the path
            move_from_hover = [move_from[0], move_from[1], self.hover_height]
            move_to_hover = [move_to[0], move_to[1], self.hover_height]
            dead_zone_hover = [self.dead_zone[0], self.dead_zone[1], self.hover_height]

            current_position = [franka.x, franka.y, franka.z]

            moves = np.array([[[current_position, self.rest], [current_position, move_to_hover],
                               [current_position, died]],
                              [[current_position, move_to_hover],
                               [current_position, dead_zone_hover]],
                              [[current_position, move_from_hover], [current_position, move_from]],
                              [[current_position, move_from_hover],
                               [current_position, move_to_hover], [current_position, move_to]],
                              [[current_position, move_to_hover], [current_position, self.rest]]])

        else:
            raise ValueError("The length of the chess move was invalid; not 1 or 2 items")

        # # plot the trajectory
        # if self.visual:
        #     fig = plt.figure()
        #     ax3d = fig.add_subplot(111, projection='3d')

        # if self.visual:
        #     # Separate into xyz
        #     x = [coord[0] for coord in path]
        #     y = [coord[1] for coord in path]
        #     z = [coord[2] for coord in path]
        #
        #     # getting board points
        #     board_x = [coord[0] for coord in self.board]
        #     board_y = [coord[1] for coord in self.board]
        #     board_z = [coord[2] for coord in self.board]
        #     # add the first point to the end of the list to create polygon
        #     board_x.append(self.board[0][0])
        #     board_y.append(self.board[0][1])
        #     board_z.append(self.board[0][2])
        #
        #     # plotting
        #     fig = plt.figure()
        #     ax3d = fig.add_subplot(111, projection='3d')
        #
        #     # plot the board
        #     ax3d.plot(board_x, board_y, board_z, 'y')
        #     # plot important points
        #     ax3d.plot([self.rest[0]], [self.rest[1]], [self.rest[2]], 'g*')
        #     ax3d.plot([self.dead_zone[0]], [self.dead_zone[1]], [self.dead_zone[2]], 'g*')
        #     # plot the untouched path
        #     ax3d.plot(x, y, z, 'r')
        #     plt.show()

        return moves

    def input_chess_move(self, arm_object, chess_move_an):
        """
        After new move is fetched from the game engine, the result is passed to this
        function so that a trajectory may be generated. This is then executed on FRANKA.

        :param arm_object: Takes object of Franka arm control class.
        :param chess_move_an: Takes chess move generated from chess engine.
        """
        ungrip_dim = 0.045
        gripper_delay = 0.5

        if len(chess_move_an) > 1:
            chess_play = chess_move_an[1][1]
        else:
            chess_play = chess_move_an[0][1]

        # acceleration improvement if move does not involve protected ranks
        ranks = ['8']
        if (chess_play[1] not in ranks) and (chess_play[3] not in ranks):
            a = 0.08
        else:
            a = 0.02

        moves = self.generate_moves(chess_move_an, arm_object)
        current_position = [arm_object.x, arm_object.y, arm_object.z]

        for i, series in enumerate(moves):
            series = np.array(series)

            for path in series:
                path = np.array(path)
                # we update the starting point to the position of the FRANKA arm currently
                path[0] = current_position

                # # plot 2 points
                # if self.visual:
                #     ax3d.plot(path[:, 0], path[:, 1], path[:, 2], 'r*')
                #     ax3d.plot(path[:, 0], path[:, 1], path[:, 2], 'r')

                motion_plan = self.apply_trapezoid_vel(path, acceleration=a)

                arm_object.send_trajectory(motion_plan)
                # for x, y, z, speed in motion_plan:
                #     # print(x,y,z,speed)
                #     arm.move_to(x, y, z, speed)
                #     time.sleep(0.005)  # control loop

                # update current position
                current_position = [arm_object.x, arm_object.y, arm_object.z]

            # gripper starts here

            if len(chess_move_an) == 1:  # if not piece died
                piece = chess_move_an[0][0].lower()  # which is the piece?
                dims = self.piece_dims[piece]  # height and grip dims
                grip_dim = dims[0]  # grip dims
                if i == 0:
                    # grip
                    time.sleep(gripper_delay)
                    arm_object.move_gripper(grip_dim, 0.1)
                    time.sleep(gripper_delay)
                elif i == 1:
                    # release
                    time.sleep(gripper_delay)
                    arm_object.move_gripper(ungrip_dim, 0.1)
                    time.sleep(gripper_delay)

            elif len(chess_move_an) == 2:
                piece = chess_move_an[0][0].lower()
                dims = self.piece_dims[piece]
                grip_dim = dims[0]

                # PICKING UP DEAD PIECE
                if i == 0:
                    # grip
                    time.sleep(gripper_delay)
                    arm_object.move_gripper(grip_dim, 0.1)
                    time.sleep(gripper_delay)

                # DROPPING OFF DEAD PIECE AT DEAD ZONE
                elif i == 1:
                    # release
                    time.sleep(gripper_delay)
                    arm_object.move_gripper(ungrip_dim, 0.1)
                    time.sleep(gripper_delay)

                # PICKING UP NEW PIECE
                elif i == 2:
                    piece = chess_move_an[1][0]  # change piece dims for second grip
                    dims = self.piece_dims[piece]
                    grip_dim = dims[0]

                    # grip
                    time.sleep(gripper_delay)
                    arm_object.move_gripper(grip_dim, 0.1)
                    time.sleep(gripper_delay)

                # DROPPING OFF NEW PIECE TO ITS DESTINATION
                elif i == 3:
                    # release
                    time.sleep(gripper_delay)
                    arm_object.move_gripper(ungrip_dim, 0.1)
                    time.sleep(gripper_delay)


if __name__ == '__main__':
    from franka.franka_control_ros import FrankaRos
    arm = FrankaRos(init_ros_node=True)  # create an arm object for testing motion generation

    planner = MotionPlanner(visual=False, debug=True)

    # EXAMPLE MOTION TO TEST IF PARTS ARE WORKING
    # example_path = np.array([[arm.x, arm.y, arm.z], [arm.x, arm.y + 0.2, arm.z]])  # move +y
    # motion_plan = planner.apply_trapezoid_vel(example_path)
    # arm.send_trajectory(motion_plan)

    if True:  # For debugging purposes
        # noinspection PyUnresolvedReferences
        import rospy

        try:
            # EXAMPLE CHESS MOVES
            chess_move = [('n', 'g8f6')]
            # chess_move = [("p", "e4"), ("p", "d5e4")]
            planner.input_chess_move(arm, chess_move)

            time.sleep(2)
            print("finished motion.")

        except rospy.ROSInterruptException as e:
            print(e)
