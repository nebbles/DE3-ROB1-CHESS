from motion import Trajectory
from franka.franka_control import FrankaControl

vector_list = Trajectory.get_test_trajectory()

new_vectors = []
for vector in vector_list:
    dx = float(vector[0])/1000.0
    dy = float(vector[1])/1000.0
    dz = float(vector[2])/1000.0
    new_vectors.append([dx, dy, dz])

print("New vectors (mm converted to m): ", new_vectors)

arm = FrankaControl(debug_flag=True)
for move in new_vectors:
    x = move[0]
    y = move[1]
    z = move[2]
    arm.move_relative(dx=x, dy=y, dz=z)
