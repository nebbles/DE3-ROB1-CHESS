from motion import Trajectory
from franka.franka_control import FrankaControl
from numpy import invert

trajectory = Trajectory.output([("r", "b4"),("r", "a1a2")] ,visual_flag=False)

rest = trajectory[0]

arm = FrankaControl(debug=True)
rest_FRANKA = arm.get_end_effector_pos()

T = rest_FRANKA * invert(rest)


