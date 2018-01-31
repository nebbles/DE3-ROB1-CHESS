from numpy import array, sqrt, arange, linalg
from sympy import symbols, diff

# start and goal values
x1, y1, z1 = [1,2,3]
x2, y2, z2 = [4,5,6]

start = array([x1, y1, z1])
destination = array([x2, y2, z2])

## time span and resolution
dt = 0.01 # seconds
time = arange(0,4, dt) # seconds
t = symbols('t')

#Jacobian matrix
jacobian = [[1,2,3,4,5,6,7],[8,9,5,6,8,3,4],[5,7,4,6,8,8,8]] # GET FROM DOCUMENTATION

# Movement functions
def up():
    x = 0
    y = 0
    z = 100*t #mm
    trajectory = array([x, y, z])
    return trajectory

def down():
    x = 0
    y = 0
    z = -200*t #mm
    trajectory = array([x, y, z])
    return trajectory

def across(x1, y1, x2, y2):
    length = sqrt((x2 - x1)**2 + (y2-y1)**2)
    x = (x2-x1)*t ################### WRONG HELP #########
    y = (y2-y1)*t
    z = 0
    trajectory = array([x, y, z])
    return trajectory

# Inverse kinematics

trajectory_up = up()
trajectory_down = down()
trajectory_across = across(x1, y1, x2, y2)

v_up = diff(trajectory_up, t)
v_down = diff(trajectory_down,t)
v_across = diff(trajectory_across, t)

# starting angles - Franka's base position

print('jac:', jacobian)
print('v_up:', v_up)

# Up loop
for i in time:
    # update jacobian matrix
    j_current = jacobian.subs(t, i)
    v_current = v_up.subs(t, i)

    j_inv = j_current.pinv()
    q_dot = j_inv * v_current
    print(j_inv)
    print(v_current)

# For each up, down, across:
    # Differentitiate trajectory equations wrt time to find velocity equation (symbolic)

    # Inverse kinematics to find joint angles
        # update teh jacobian matrix
        # update the current end effector velocity

        # invert the jacobian
        # angular velocity = inv jacobian * end effector velocity
        # numerical integration to find joint angles
        # Append angles, angular velocities, end effector

    # join the up, down, across lists to create a total




    # REdundancy resolution
        # Moore Penrose Pseudo-Inverse minimises KE (OLS)
        # pinv on MATLAB