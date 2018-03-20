#!/usr/bin/env python
# Benedict Greenberg, March 2018
# http://github.com/nebbles
#
# Make sure that you are running 'roscore' and 'franka_controller_sub' before attempting to
# move the robot using this class, as it only publishes to ROS topics to interact with the Franka.
from __future__ import print_function
import time
import os
import argparse
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension


class FrankaRos:
    def __init__(self, log=False, ip='192.168.0.88', debug=False, init_ros_node=False):
        """Initialisation of the FrankaRos class.

        Sets boolean flags such as logging and print statements for debugging. Sets the IP
        address and path of the file for calling C++ binaries. Sets up a number of ROS message
        structures needed to encode the commands being sent via our ROS topics.

        It also initialises a rospy node and three publishers for:
        * moving the end effector to a 3D point with a certain speed
        * moving the grippers to grasp an object of a certain size with a force and speed
        * moving the grippers to a certain width with a speed
        """
        self.log = log
        self.ip_address = ip
        self.debug = debug
        self.path = os.path.dirname(os.path.realpath(__file__))  # gets working dir of this file

        # arm movement
        self.target_coords = Float64MultiArray()  # the three absolute target coordinates
        self.target_coords.layout.dim.append(MultiArrayDimension())  # coordinates
        self.target_coords.layout.dim[0].label = "coordinates"
        self.target_coords.layout.dim[0].size = 3

        self.target_coords.layout.dim.append(MultiArrayDimension())  # speed
        self.target_coords.layout.dim[1].label = "speed"
        self.target_coords.layout.dim[1].size = 1

        # gripper
        self.target_gripper = Float64MultiArray()  # gripper stuff
        self.target_gripper.layout.dim.append(MultiArrayDimension())  # gripper
        self.target_gripper.layout.dim[0].label = "gripper"
        self.target_gripper.layout.dim[0].size = 3  # width, speed, force

        # ros initiation
        if init_ros_node:
            if self.debug:
                print("Starting ROS node: franka_control_ros")
            rospy.init_node('franka_control_ros', anonymous=True)
        self.pub_move_to = None
        self.pub_grasp = None
        self.pub_move_grip = None

        self.x = None
        self.y = None
        self.z = None
        self.pos_sub = None
        self.start_subscriber()
        time.sleep(0.5)
        print("Waiting for subscriber to return initial Franka position. You may need to check that 'franka_controller_sub' is running.")
        while self.z is None:
            time.sleep(0.1)
        print("Initial Franka position found: ", [self.x, self.y, self.z])

    def start_subscriber(self):
        self.pub_move_to = rospy.Publisher('franka_move_to', Float64MultiArray, queue_size=1)
        self.pub_grasp = rospy.Publisher('franka_gripper_grasp', Float64MultiArray, queue_size=0)
        self.pub_move_grip = rospy.Publisher('franka_gripper_move', Float64MultiArray, queue_size=0)
        
        self.pos_sub = rospy.Subscriber('franka_current_position', Float64MultiArray, self.subscribe_position_callback, queue_size=1)

    def stop_subscriber(self):
        self.pub_move_to.unregister()
        self.pub_grasp.unregister()
        self.pub_move_grip.unregister()
        self.pos_sub.unregister()

    def subscribe_position_callback(self, data):
        # TODO docstring
        if self.log:
            rospy.loginfo(data.data)
        self.x = data.data[0]
        self.y = data.data[1]
        self.z = data.data[2]

    def move_to(self, x, y, z, speed):
        """Moves robot end effector to desired coordinates (in robot reference frame) given a
        target velocity. The robot will not necessarily travel at this speed due to safety
        controls in the subscriber.

        :param x: float value position in robot reference axis
        :param y: float value position in robot reference axis
        :param z: float value position in robot reference axis
        :param speed: float value desired speed to target position
        """
        self.target_coords.data = [float(x), float(y), float(z), float(speed)]
        if self.log:
            rospy.loginfo("franka_move_to: " + str(self.target_coords.data))
        self.pub_move_to.publish(self.target_coords)

    def send_trajectory(self, trajectory):
        # TODO docstring
        for x, y, z, speed in trajectory:
            self.move_to(x, y, z, speed)
            time.sleep(0.005)  # control loop

    def move_relative(self, dx, dy, dz, speed):
        """Moves robot end effector in desired direction (in robot reference frame) given a
        target displacement and speed to travel at. The robot will not necessarily travel at this
        speed due to safety controls in the subscriber.

        :param dx: float value displacement in robot reference axis
        :param dy: float value displacement in robot reference axis
        :param dz: float value displacement in robot reference axis
        :param speed: float value desired speed to target displaced position
        """
        x, y, z = self.get_position()
        self.target_coords.data = [float(x+dx), float(y+dy), float(z+dz), float(speed)]
        if self.log:
            rospy.loginfo("franka_move_relative: " + str(self.target_coords.data))
        self.pub_move_to.publish(self.target_coords)

    def move_gripper(self, width, speed):
        """Set gripper width by assigning ``width`` with a desired speed to move at. Note this
        must be called before grasping if gripper fingers are too close together ``grasp()`` call.

        0 width defined == 2.2 cm difference real-world

        :param width: desired distance between prongs on end-effector (in millimetres)
        :param speed: desired speed with which to move the grippers
        """
        self.target_gripper.data = [float(width), float(speed)]
        if self.log:
            rospy.loginfo("franka_gripper_move: " + str(self.target_gripper.data))
        self.pub_move_grip.publish(self.target_gripper)

    def grasp(self, object_width, speed, force):
        """Grasp an object in the grippers. Note that this can only be called if the object width is
        smaller than the current distance between the grippers.

        The grippers attempt to move to the width of the object defined with speed defined,
        and then proceed to apply a defined force against the object. If no object is present it
        evaluates as failed and moves to next task.

        0 width defined == 2.2 cm difference real-world

        :param object_width: width of target object to grab (float)
        :param speed: with which to move the gripper to object width (float)
        :param force: to apply to the object once the grippers are at object width (float)
        """
        self.target_gripper.data = [float(object_width), float(speed), float(force)]
        if self.log:
            rospy.loginfo("franka_gripper_grasp: " + str(self.target_gripper.data))
        self.pub_grasp.publish(self.target_gripper)

    def get_position(self):  # TODO update docstring
        """Get x, y, z position of end-effector and returns it to caller.

        :return: list as [x, y, z]
        """
        return [self.x, self.y, self.z]


def example_position():
    """Used to test if position reporting is working from Arm.

    It will repeatedly print the full arm position data and the XYZ position of the end-effector.
    To use this test, add the ``-p`` or ``--position-example`` flag to the command line.
    """
    arm = FrankaRos(debug=True)
    while True:
        data = arm.get_position()
        print("End effector position:")
        print("X: ", data[0])
        print("Y: ", data[1])
        print("Z: ", data[2])


def example_movement():  # TODO replace or remove functionality
    """Used to test if the arm can be moved with gripper control.

    Function moves the arm through a simple motion plan and then tried moving the grippers.
    To use this test, add the ``-m`` or ``--motion-example`` flag to the command line.
    """
    pass
    # try:
    #     franka = FrankaRos(debug=True)
    #
    #     motion_plan = []
    #     resolution = 100
    #     for i in range(1, resolution):
    #         motion_plan.append((0.4+i/resolution, 0.4+i/resolution, 0.4+i/resolution, 0.100))
    #
    #     # print(motion_plan)
    #     for x, y, z, speed in motion_plan:
    #         franka.move_to(x, y, z, speed)
    #         time.sleep(0.1)  # 10 Hz control loop
    #
    #     # now test the grippers are operational
    #     width = 0.06  # 2.2 cm = 0 width
    #     speed = 0.1
    #     force = 1
    #     franka.move_gripper(width, speed)
    #     time.sleep(5)
    #     width = 0.037  # 2.2 cm = 0 width
    #     franka.grasp(width, speed, force)
    #     time.sleep(5)
    #     width = 0.06  # 2.2 cm = 0 width
    #     franka.move_gripper(width, speed)
    #
    # except rospy.ROSInterruptException:
    #     pass


if __name__ == '__main__':
    # TODO clear main up
    parser = argparse.ArgumentParser(description='Control Franka Arm with Python.')
    # parser.add_argument('-m', '--motion-example', action='store_true',
    #                     help='run example motion function')
    parser.add_argument('-p', '--position-example', action='store_true',
                        help='run example position reading function')
    args = parser.parse_args()  # get command line arguments

    try:
        # if args.motion_example:
        #     example_movement()
        if args.position_example:
            example_position()
        else:
            print("Try: franka_control.py --help")
    except KeyboardInterrupt:
        print("\nExiting...")
