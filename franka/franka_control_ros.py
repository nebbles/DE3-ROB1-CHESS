#!/usr/bin/env python
# Benedict Greenberg, March 2018
# http://github.com/nebbles
#
# Make sure that you are running 'roscore' and 'franka_controller_sub' before attempting to
# move the robot using this class, as it only publishes to ROS topics to interact with the Franka.

from __future__ import print_function
import time
import os
import subprocess
import argparse
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension


class FrankaRos:
    def __init__(self, log=True, ip='192.168.0.88', debug=False):
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
        rospy.init_node('franka_python_node', anonymous=True)
        self.pub_move_to = rospy.Publisher('franka_move_to', Float64MultiArray, queue_size=1)
        self.pub_grasp = rospy.Publisher('franka_gripper_grasp', Float64MultiArray, queue_size=0)
        self.pub_move_grip = rospy.Publisher('franka_gripper_move', Float64MultiArray, queue_size=0)

        time.sleep(0.5)

    def move_to(self, x, y, z, speed):  # todo add docstring
        self.target_coords.data = [float(x), float(y), float(z), float(speed)]
        if self.log:
            rospy.loginfo("franka_move_to: " + str(self.target_coords.data))
        self.pub_move_to.publish(self.target_coords)

    # TODO add move relative

    def move_gripper(self, width, speed):  # todo add docstring
        self.target_gripper.data = [float(width), float(speed)]  # [0.1, 0.1, 0.1, 0.10]
        if self.log:
            rospy.loginfo("franka_gripper_move: " + str(self.target_gripper.data))
        self.pub_move_grip.publish(self.target_gripper)

    def grasp(self, object_width, speed, force):  # todo add docstring
        self.target_gripper.data = [float(object_width), float(speed), float(force)]
        if self.log:
            rospy.loginfo("franka_gripper_grasp: " + str(self.target_gripper.data))
        self.pub_grasp.publish(self.target_gripper)

    def get_joint_positions(self):  # todo convert
        """Gets current joint positions for Franka Arm.

        This will return a list of lists of joint position data. This data structure has not been
        documented yet and is not recommended for use.
        """

        program = './get_joint_positions'  # set executable to be used
        command = [program, self.ip_address]
        command_str = " ".join(command)

        if self.debug:
            print("Working directory: ", self.path)
            print("Program: ", program)
            print("IP Address of robot: ", self.ip_address)
            print("Command being called: ", command_str)
            print("Running FRANKA code...")

        process = subprocess.Popen(command, cwd=self.path, stdout=subprocess.PIPE)
        out, err = process.communicate()  # this will block until received
        decoded_output = out.decode("utf-8")

        import ast
        string_list = decoded_output.split("\n")

        converted_list = []
        for idx, lit in enumerate(string_list):
            x = lit
            x = ast.literal_eval(x)
            converted_list.append(x)
            if idx == 7:
                break  # We have parsed all 8 items from ./get_joint_positions

        return converted_list


def example_position():
    """Used to test if position reporting is working from Arm.

    It will repeatedly print the full arm position data and the XYZ position of the end-effector.
    To use this test, add the ``-p`` or ``--position-example`` flag to the command line.
    """
    arm = FrankaRos(debug=True)
    while True:
        data = np.array(arm.get_joint_positions())
        print(data)

        print("End effector position:")
        print("X: ", data[7, 12])
        print("Y: ", data[7, 13])
        print("Z: ", data[7, 14])


def example_movement():
    """Used to test if the arm can be moved with gripper control.

    Function moves the arm through a simple motion plan and then tried moving the grippers.
    To use this test, add the ``-m`` or ``--motion-example`` flag to the command line.
    """
    try:
        franka = FrankaRos(debug=True)

        motion_plan = []
        resolution = 100
        for i in range(1, resolution):
            motion_plan.append((0.4+i/resolution, 0.4+i/resolution, 0.4+i/resolution, 0.100))

        # print(motion_plan)
        for x, y, z, speed in motion_plan:
            franka.move_to(x, y, z, speed)
            time.sleep(0.1)  # 10 Hz control loop

        # TODO install this example code for gripper movement

        # """This is an example method for oper"""
        # width = 0.06  # 2.2 cm = 0 width
        # speed = 0.1
        # force = 1
        #
        # self.target_gripper.data = [width, speed]  # [0.1, 0.1, 0.1, 0.10]
        # rospy.loginfo("franka_gripper_move: " + str(self.target_gripper.data))
        # self.pub_move_grip.publish(self.target_gripper)
        #
        # time.sleep(5)
        #
        # width = 0.037  # 2.2 cm = 0 width
        # self.target_gripper.data = [width, speed, force]
        # rospy.loginfo("franka_gripper_grasp: " + str(self.target_gripper.data))
        # self.pub_grasp.publish(self.target_gripper)
        #
        # time.sleep(5)
        #
        # width = 0.06  # 2.2 cm = 0 width
        # self.target_gripper.data = [width, speed, force]
        # rospy.loginfo("franka_gripper_move: " + str(self.target_gripper.data))
        # self.pub_move_grip.publish(self.target_gripper)

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Control Franka Arm with Python.')
    parser.add_argument('-m', '--motion-example', action='store_true',
                        help='run example motion function')
    parser.add_argument('-p', '--position-example', action='store_true',
                        help='run example position reading function')
    args = parser.parse_args()  # Get command line arguments

    try:
        if args.motion_example:
            example_movement()
        elif args.position_example:
            example_position()
        else:
            print("Try: franka_control.py --help")
    except KeyboardInterrupt:
        print("\nExiting...")
