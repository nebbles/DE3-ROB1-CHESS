import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


def calibrate_cube(centre_point):
    """function generates an array of 8 x,y,z coordinates for calibration based on an end-effector (1x3) input position"""

    x, y, z = centre_point[0], centre_point[1], centre_point[2]
    # Initial Cube
    p_cube = np.array([[-0.1, -0.1, -0.1], [-0.1, 0.1, -0.1], [-0.1, 0.1, 0.1], [-0.1, -0.1, 0.1], [0.1, -0.1, -0.1], [0.1, 0.1, -0.1], [0.1, -0.1, 0.1], [0.1, 0.1, 0.1]])

    # print(p_cube)

    p_cube_x = []
    p_cube_y = []
    p_cube_z = []
    p_cube_x_rxyz = []
    p_cube_y_rxyz = []
    p_cube_z_rxyz = []

    cube_output = np.zeros((8, 3))
    cube_output_x = []
    cube_output_y = []
    cube_output_z = []

    # Transformation matrices
    theta = np.radians(30)
    cos = np.cos(theta)
    sin = np.sin(theta)
    rot_x = np.array([[1, 0, 0], [0, cos, -sin], [0, sin, cos]])
    rot_y = np.array([[cos, 0, sin], [0, 1, 0], [-sin, 0, cos]])
    rot_z = np.array([[cos, -sin, 0], [sin, cos, 0], [0, 0, 1]])

    # Multiplying Initial Cube by Transformation matrices in X, Y & Z
    p_cube_rot_x = p_cube.dot(rot_x)
    p_cube_rot_xy = p_cube_rot_x.dot(rot_y)
    p_cube_rot_xyz = p_cube_rot_xy.dot(rot_z)

    for i in range(len(p_cube)):
        x_tr = x + p_cube_rot_xyz[i, 0]
        y_tr = y + p_cube_rot_xyz[i, 1]
        z_tr = z + p_cube_rot_xyz[i, 2]
        cube_output[i, 0] = x_tr
        cube_output[i, 1] = y_tr
        cube_output[i, 2] = z_tr
        cube_output_x.append([x_tr])
        cube_output_y.append([y_tr])
        cube_output_z.append([z_tr])

    # Separating X, Y, Z coordinates into separate lists for 3D Plot
    for i in range(len(p_cube)):
        x_1 = p_cube[i, 0]
        p_cube_x.append(x_1)
        y_1 = p_cube[i, 1]
        p_cube_y.append(y_1)
        z_1 = p_cube[i, 2]
        p_cube_z.append(z_1)

    for i in range(len(p_cube)):
        x_rx = p_cube_rot_xyz[i, 0]
        p_cube_x_rxyz.append(x_rx)
        y_rx = p_cube_rot_xyz[i, 1]
        p_cube_y_rxyz.append(y_rx)
        z_rx = p_cube_rot_xyz[i, 2]
        p_cube_z_rxyz.append(z_rx)

    # 3D Plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(p_cube_x, p_cube_y, p_cube_z)
    # ax.scatter(p_cube_x_rxyz, p_cube_y_rxyz, p_cube_z_rxyz)
    ax.scatter(x, y, z)
    ax.scatter(cube_output_x, cube_output_y, cube_output_z)

    plt.show()

    # print(cube_output)

    return cube_output


if __name__ == '__main__':
    c = calibrate_cube((1, 1, 1))
    print('output positions =', c)
