# Convert game engine coordinates to real world x y coords class
    # not our problem

start = [x1; y1; z1];
destination = [x2, y2, z2];

# Trajectory of end effector planning class - outputs 3 trajectory equations (symbolic)
    # up
    # down
    # across inputs: pythagoras on given x and y coords

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