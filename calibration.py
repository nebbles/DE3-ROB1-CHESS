import cv2
import imutils
import matplotlib.pyplot as plt
import numpy as np


def run_calibration(arm_object, planner_object, cam_feed):
    """
    Runs calibration moving the end-effector between the 8 vertices of the cube (x, y, z),
    then detecting the marker at each of these 8 steps,
    applying the marker offset and producing the array of u, v, w positions.

    :param arm_object:
    :param planner_object:
    :param cam_feed:
    :return: 8x3 array with marker coordinates (camera frame) and 8x3 array with respective franka coordinates (vertices of cube)
    """
    franka_pos = arm_object.get_position()
    moves = generate_cube(franka_pos)
    marker_coordinates = np.zeros((8, 3))
    franka_coordinates = np.zeros((8, 3))

    for i in range(1, np.shape(moves)[0]):  # not lengths number of rows needs to be fixed
    
        cur_pos = arm_object.get_position()
        complete_trajectory = planner_object.apply_trapezoid_vel([cur_pos, moves[i]])
    
        arm_object.send_trajectory(complete_trajectory)
        # delay?
    
        franka_x, franka_y, franka_z = arm_object.get_position()
        franka_coordinates[i-1, 0] = franka_x
        franka_coordinates[i-1, 1] = franka_y
        franka_coordinates[i-1, 2] = franka_z

        rgb, depth = cam_feed.get_frames()
        current_marker_pos = find_cross_auto(rgb)  # change data type tupple to array

        current_marker_pos = find_depth(depth, current_marker_pos)
        # fix for the offset between actual location of marker and desired
        current_marker_pos = fix_marker_offset(current_marker_pos)

        marker_coordinates[i-1, 0] = current_marker_pos[0]
        marker_coordinates[i-1, 1] = current_marker_pos[1]
        marker_coordinates[i-1, 2] = current_marker_pos[2]

    return marker_coordinates, franka_coordinates


def generate_cube(centre_point):
    """
    Generates an array of 8 x,y,z coordinates for calibration based on an end-effector (1x3)
    input position.

    :param centre_point:
    :return:
    """
    x, y, z = centre_point[0], centre_point[1], centre_point[2]
    # Initial Cube
    p_cube = np.array(
        [[-0.05, -0.05, -0.05], [-0.05, 0.05, -0.05], [-0.05, 0.05, 0.05], [-0.05, -0.05, 0.05],
         [0.05, -0.05, -0.05], [0.05, 0.05, -0.05], [0.05, -0.05, 0.05], [0.05, 0.05, 0.05]])

    p_cube_x = []
    p_cube_y = []
    p_cube_z = []
    p_cube_x_rxyz = []
    p_cube_y_rxyz = []
    p_cube_z_rxyz = []

    cube_output = np.zeros((9, 3))
    cube_output_x = []
    cube_output_y = []
    cube_output_z = []

    cube_output[0, 0] = x
    cube_output[0, 1] = y
    cube_output[0, 2] = z

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
        cube_output[i+1, 0] = x_tr
        cube_output[i+1, 1] = y_tr
        cube_output[i+1, 2] = z_tr
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

    return cube_output  # 9x3 array


def detect(c):
    """
    Used by find_cross function to detect the edges and vertices of possible polygons in an image.

    :param c:
    :return:
    """
    peri = cv2.arcLength(c, True)
    area = cv2.contourArea(c)
    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
    return len(approx), peri, area


def find_cross_manual(frame):
    """
    Uses open CV2 to detect a red cross in a photo. This function applies a red mask,
    finds contours, finds the number of vertexes in polygons, and selects the shape with the
    appropriate number of vertices, area and perimeter. It then finds the centre of the cross and
    returns its coordinates. Manual detection mode allows users to double check that if one marker
    is detected that the correct one has been selected, and if multiple detectors are selected they are
    able to enter the number of the correct marker.

    :param frame:
    :return: x,y coordinates of the red cross marker in camera frame
    """

    markers = []
    markers_tuple = []

    while 1:
        # img = cv2.imread('test_2.jpg')

        img = frame
        img_sized = imutils.resize(img, width=640)
        img_hsv = cv2.cvtColor(img_sized, cv2.COLOR_BGR2HSV)

        # cv2.imshow('frameb', imgsized)

        lower_red1 = np.array([170, 70, 50])
        upper_red1 = np.array([180, 255, 255])
        lower_red2 = np.array([0, 70, 50])
        upper_red2 = np.array([10, 200, 200])

        mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(img_hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        # cv2.imshow('mask', mask)

        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if imutils.is_cv2() else contours[1]

        for c in contours:
            shape = detect(c)

            if 14 > shape[0] > 10 and 800 > shape[1] > 50 and 15000 > shape[2] > 100:
                # if 14 > shape[0] > 10:
                m = cv2.moments(c)
                c_x = int((m["m10"] / m["m00"]))
                c_y = int((m["m01"] / m["m00"]))
                cv2.circle(img_sized, (c_x, c_y), 3, (0, 255, 255), -1)
                markers.append(c_x)
                markers.append(c_y)
                markers_tuple.append((c_x, c_y))

                cv2.drawContours(img_sized, [c], -1, (0, 255, 0), 2)

                for i in range(len(markers_tuple)):
                    cv2.putText(img_sized, str(i), markers_tuple[i], cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 0, 0), 2)

                cv2.imshow("Image", img_sized)

        # print markers

        # pts = np.array(markers)
        # cv2.polylines(imgsized, [pts], True, (255, 0, 255), 3)
        # cv2.imshow("Image", imgsized)

        cv2.waitKey(0)
        break

    chosen_marker = None
    if len(markers_tuple) == 1:
        double_check = input("Was the correct marker selected? Press 1 for YES or 9 for NO")
        if double_check == 1:
            # print markers
            chosen_marker = markers
            pass
        elif double_check == 9:
            print "Shape detection parameters require adjusting"

    else:
        valid = input("Enter number of correct marker")
        chosen_marker = markers_tuple[int(valid)]
        # print chosen_marker

    print chosen_marker
    return chosen_marker


def find_cross_auto(frame):
    """
    Uses open CV2 to detect a red cross in a photo. This function applies a red mask,
    finds contours, finds the nuber of vertexes in polygons, and selects the shape with the
    appropriate number of vertices, area and perimeter. It then finds the centre of the cross and
    returns its coordinates. The automatic mode the detected marker is automatically returned,
    however if multiple are detected they are able to switch to manual mode.

    :param frame:
    :return: x,y coordinates of the red cross marker in camera frame
    """

    markers = []
    markers_tuple = []

    while 1:
        # img = cv2.imread(frame)
        img = frame
        img_sized = imutils.resize(img, width=640)
        # img_sized = imutils.resize(img, width=640) try without resizing
        img_hsv = cv2.cvtColor(img_sized, cv2.COLOR_BGR2HSV)

        # cv2.imshow('frameb', imgsized)

        lower_red1 = np.array([170, 70, 50])
        upper_red1 = np.array([180, 255, 255])
        lower_red2 = np.array([0, 70, 50])
        upper_red2 = np.array([10, 255, 255])

        mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(img_hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        # cv2.imshow('mask', mask)

        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if imutils.is_cv2() else contours[1]

        for c in contours:

            shape = detect(c)

            if 14 > shape[0] > 10 and 800 > shape[1] > 50 and 15000 > shape[2] > 100:
                # print 'I am here'
                # print shape
                # change these values to match geometry of cross
                # if 14 > shape[0] > 10 and 200 > shape[1] > 50 and 820 > shape[2] > 100:
                m = cv2.moments(c)

                c_x = int((m["m10"] / m["m00"]))
                c_y = int((m["m01"] / m["m00"]))
                cv2.circle(img_sized, (c_x, c_y), 3, (0, 255, 255), -1)
                markers.append(c_x)
                markers.append(c_y)
                markers_tuple.append((c_x, c_y))

                cv2.drawContours(img_sized, [c], -1, (0, 255, 0), 2)

                for i in range(len(markers_tuple)):
                    cv2.putText(img_sized, str(i), markers_tuple[i], cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 0, 0), 2)

                cv2.imshow("Image", img_sized)

        cv2.waitKey(0)

        # pts = np.array(markers)
        # cv2.polylines(imgsized, [pts], True, (255, 0, 255), 3)
        # cv2.imshow("Image", imgsized)

        chosen_marker = markers
        # print chosen_marker
        if len(markers_tuple) > 1:
            print "More than one cross detected, please switch to manual calibration"
            break
        else:
            break

    raw_input("press enter to continue")
    cv2.destroyAllWindows()

    print chosen_marker
    return chosen_marker


def find_depth(img, coordinate):
    """
    Finds depth of marker centre in a depth image.

    :param img:
    :param coordinate:
    :return:
    """
    # img = cv2.imread('test_3.jpg') #might not be needed if already reading a cv image
    depth = img[(coordinate[1]), (coordinate[0])]
    coordinates = coordinate[0], coordinate[1], depth[0]

    return coordinates


def fix_marker_offset(coordinates):
    """
    Relates coordinates of centre of cross to Franka end-effector position.

    **Note** Currently this is not implemented.
    """
    # TODO add the fixed marker offset
    return coordinates


if __name__ == '__main__':
    # noinspection PyUnresolvedReferences
    import rospy
    import camera_subscriber
    from franka.franka_control_ros import FrankaRos
    from motion import MotionPlanner

    rospy.init_node('franka_python_node', anonymous=True)
    feed = camera_subscriber.CameraFeed()
    arm = FrankaRos()
    
    planner = MotionPlanner(visual=False, debug=True)

    run_calibration(arm, planner, feed)
