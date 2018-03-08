from motion import motion_planning

# Instantiate the motion class
FRANKA = motion_planning.MotionPlanning()

#FRANKA.make_move([("r", "a1a6")], visual_flag=True)
FRANKA.make_move([("r", "b4"), ("r", "a1b4")], visual_flag=True)