from motion import Trajectory
from franka.franka_control import FrankaControl

# collect data from 4 FRANKA frame points
arm = FrankaControl(debug_flag=False)  # debug=True)

def callib(arm):#trajectory_chess):
    """Function to gather the FRANKA frame coordinates of the key positions"""

    print("For the following commands, place the FRANKA end effector in the centre of the relevant square\n")

    valid = input("Move the arm to A1 corner")
    if valid == "":
        pass
    else:
        print('YOU FUCKED UP')
    try:
        corner_A1_FRANKA = arm.get_end_effector_pos()
    except:
        corner_A1_FRANKA = [0,0,0]

    valid = input("Move the arm to A8 corner")
    if valid == "":
        pass
    else:
        print('YOU FUCKED UP')
    try:
        corner_A8_FRANKA = arm.get_end_effector_pos()
    except:
        corner_A8_FRANKA = [0,908,0]

    valid = input("Move the arm to H8 corner")
    if valid == "":
        pass
    else:
        print('YOU FUCKED UP')
    try:
        corner_H8_FRANKA = arm.get_end_effector_pos()
    except:
        corner_H8_FRANKA = [908, 908, 0];

    valid = input("Move the arm to H1 corner")
    if valid == "":
        pass
    else:
        print('YOU FUCKED UP')
    try:
        corner_H1_FRANKA = arm.get_end_effector_pos()
    except:
        corner_H1_FRANKA = [908,0,0]


    valid = input("Move arm to the desired hover height")
    if valid == "":
        pass
    else:
        print('YOU FUCKED UP')
    try:
        hover_FRANKA = arm.get_end_effector_pos()
        hover = hover_FRANKA[2]
    except:
        hover_FRANKA = [0, 0, 200]
        hover = hover_FRANKA[2]

    valid = input("Move arm to the dead zone")
    if valid == "":
        pass
    else:
        print('YOU FUCKED UP')
    try:
        dead_zone = arm.get_end_effector_pos()
    except:
        dead_zone = [200, 300, 0]

    valid = input("Move arm to the rest position")
    if valid == "":
        pass
    else:
        print('YOU FUCKED UP')
    try:
        rest = arm.get_end_effector_pos()
    except:
        rest = [0, 0, 200]
        print('wtf')

    board_points = [corner_A1_FRANKA, corner_A8_FRANKA, corner_H8_FRANKA, corner_H1_FRANKA]

    return board_points, dead_zone, rest, hover

board_points, dead_zone, rest, hover = callib(arm)

trajectory = Trajectory.output([("r", "b4"),("r", "a1a2")], board_points, dead_zone, rest, hover, visual_flag=True)