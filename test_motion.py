from motion import motion_planning
#from franka.franka_control_ros import FrankaRos

#arm = FrankaRos()  # debug=True)

planner = motion_planning.MotionPlanning()

# planner.make_move([("r", "a1a6")], visual_flag=True)
# planner.make_move([("r", "b4"), ("r", "a1b4")], visual_flag=True)