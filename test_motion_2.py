from motion import Trajectory
#from franka.franka_control import FrankaControl

#arm = FrankaControl(debug_flag=False)  # debug=True)

def callib():#trajectory_chess):
    """Function to gather the FRANKA frame coordinates of the key positions"""

    print("For the following commands, place the FRANKA end effector in the centre of the relevant square\n")

    valid = input("Move the arm to A1 corner")
    if valid == "":
        pass
    try:
        corner_A1_FRANKA = arm.get_end_effector_pos()
        print('We are using actual Franka coordinates')
        print(corner_A1_FRANKA)
    except:
        corner_A1_FRANKA = [0.501, 0.199, 0.048]

    valid = input("Move the arm to A8 corner")
    if valid == "":
        pass
    try:
        corner_A8_FRANKA = arm.get_end_effector_pos()
    except:
        corner_A8_FRANKA = [0.739,0.14,0.0344]

    valid = input("Move the arm to H8 corner")
    if valid == "":
        pass
    try:
        corner_H8_FRANKA = arm.get_end_effector_pos()
    except:
        corner_H8_FRANKA = [0.71, -0.176, 0]

    valid = input("Move the arm to H1 corner")
    if valid == "":
        pass
    try:
        corner_H1_FRANKA = arm.get_end_effector_pos()
    except:
        corner_H1_FRANKA = [0.468,-0.138,-0.0156]


    valid = input("Move arm to the desired hover height")
    if valid == "":
        pass
    try:
        hover_FRANKA = arm.get_end_effector_pos()
        hover = hover_FRANKA[2]
    except:
        hover_FRANKA = [0.505,-0.0483,0.165]
        hover = hover_FRANKA[2]

    valid = input("Move arm to the dead zone")
    if valid == "":
        pass
    try:
        dead_zone = arm.get_end_effector_pos()
    except:
        dead_zone = [0.363, -0.482, 0.245]

    valid = input("Move arm to the rest position")
    if valid == "":
        pass
    try:
        rest = arm.get_end_effector_pos()
    except:
        rest = [0.29,0.125,0.818]

    board_points = [corner_A1_FRANKA, corner_A8_FRANKA, corner_H8_FRANKA, corner_H1_FRANKA]

    return board_points, dead_zone, rest, hover

# collect data from 4 FRANKA frame points
board_points, dead_zone, rest, hover = callib()

# Find trajectory
#trajectory = Trajectory.continuous_trajectory([("r", "b4"), ("r", "a1b4")], board_points, dead_zone, rest, hover, visual_flag=True)
trajectory = Trajectory.continuous_trajectory([("r", "a1a6")], board_points, dead_zone, rest, hover, visual_flag=True) # test for life

# test gripping
#trajectory = Trajectory.trajectory_and_gripping([("r", "a1a6")], board_points, dead_zone, rest, hover, visual_flag=True)
#trajectory = Trajectory.trajectory_and_gripping([("r", "b4"), ("r", "a1b4")], board_points, dead_zone, rest, hover, visual_flag=True)



# # Execute trajectory
# for move in trajectory:
#     try:
#         print(move, '\n')
#         #arm.move_absolute(move)
#     except (KeyboardInterrupt, SystemExit):
#         raise

for move in trajectory:
    try:
        if isinstance(move, list):
            print('Coord:', move)
            # execute motion
        else:
            print('\n\n', move, '\n\n')
            # execute the gripping motion
        # arm.move_absolute(move)
    except (KeyboardInterrupt, SystemExit):
        raise
