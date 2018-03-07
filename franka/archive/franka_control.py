#
# Benedict Greenberg, March 2018
"""Python Module to control the Franka Arm though simple method calls.

This module uses ``subprocess`` and ``os``.
"""
from __future__ import print_function
import subprocess
import os
import sys
import argparse
import numpy as np

if sys.version_info[:2] <= (2, 7):
    input = raw_input


class FrankaControl(object):
    """Class containing methods to control an instance of the Franka Arm.

    Will print debug information to the console when ``debug_flag=True`` argument is used. Class
    references C++ binaries to control the Franka.

    IP address of Franka in Robotics Lab already configured as default.
    """
    def __init__(self, ip='192.168.0.88', debug_flag=False):
        self.ip_address = ip
        self.debug = debug_flag
        self.path = os.path.dirname(os.path.realpath(__file__))  # gets working dir of this file

    def get_joint_positions(self):
        """Gets current joint positions for Franka Arm.

        This will return a list of lists of joint position data. This data structure has not been
        documented yet and is not recommended for use.
        """

        program = './print_joint_positions'  # set executable to be used
        command = [program, self.ip_address]
        command_str = " ".join(command)

        if self.debug:
            print("Working directory: ", self.path)
            print("Program: ", program)
            print("IP Address of robot: ", self.ip_address)
            print("Command being called: ", command_str)
            print("Running FRANKA code...")

        process = subprocess.Popen(command, stdout=subprocess.PIPE)
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

    def get_end_effector_pos(self):
        """Gets current x,y,z positions for Franka Arm end effector.

        Returns list of x,y,z values.
        """

        program = './franka_get_current_position'  # set executable to be used
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
        converted_list = ast.literal_eval(decoded_output)

        return converted_list

    def move_relative(self, dx=0.0, dy=0.0, dz=0.0):
        """Moves Franka Arm relative to its current position.

        Executes Franka C++ binary which moves the arm relative to its current position according
        to the to the delta input arguments. **Note: units of distance are in metres.**

        Returns the *exit code* of the C++ binary.
        """
        try:
            dx, dy, dz = float(dx), float(dy), float(dz)
        except ValueError:
            print("Arguments are invalid: must be floats")
            return

        dx, dy, dz = str(dx), str(dy), str(dz)

        program = './franka_move_to_relative'  # set executable to be used
        command = [program, self.ip_address, dx, dy, dz]
        command_str = " ".join(command)

        if self.debug:
            print("Working directory: ", self.path)
            print("Program: ", program)
            print("IP Address of robot: ", self.ip_address)
            print("dx: ", dx)
            print("dy: ", dy)
            print("dz: ", dz)
            print("Command being called: ", command_str)
            print("Running FRANKA code...")

        # TODO: option to suppress output
        # TODO: catch errors returned by the subprocess
        return_code = subprocess.call(command, cwd=self.path)

        if return_code == 0:
            if self.debug:
                print("No problems running ", program)
        else:
            print("Python has registered a problem with ", program)

        return return_code

    def move_absolute(self, coordinates):
        """Moves Franka Arm to an absolute coordinate position.

        Coordinates list should be in format: [x, y, z]

        This method will try to move straight to the coordinates given. These coordinates
        correspond to the internal origin defined by the Arm.

        Returns the *exit code* of the C++ binary.
        """
        if len(coordinates) > 3:
            raise ValueError("Invalid coordinates. There can only be three dimensions.")
        x, y, z = coordinates[0], coordinates[1], coordinates[2]

        try:
            x, y, z = float(x), float(y), float(z)
        except ValueError:
            print("Arguments are invalid: must be floats")
            return

        x, y, z = str(x), str(y), str(z)

        # TODO: implement safety check for target coordinates

        program = './franka_move_to_absolute'
        command = [program, self.ip_address, x, y, z]
        command_str = " ".join(command)

        if self.debug:
            print("Working directory: ", self.path)
            print("Program: ", program)
            print("IP Address of robot: ", self.ip_address)
            print("Go to x: ", x)
            print("Go to y: ", y)
            print("Go to z: ", z)
            print("Command being called: ", command_str)
            print("Running FRANKA code...")

        # TODO: option to suppress output
        # TODO: catch errors returned by the subprocess
        return_code = subprocess.call(command, cwd=self.path)

        if return_code == 0:
            if self.debug:
                print("No problems running ", program)
        else:
            print("Python has registered a problem with ", program)

        return return_code


def test_motion():
    """Used to test if module is working and can move arm.

    When module is run from the command line it will test to see if the Franka Arm can be
    controlled with a simple forward and backward motion control along the x axis. Follow on
    screen examples for usage.

    To use, call the -m or --motion-test flag from the command line.
    """

    while True:
        testing = input("Is this program being tested with the arm? [N/y]: ")
        if testing == '' or testing.lower() == 'n':
            testing = False
            break
        elif testing.lower() == 'y':
            testing = True
            break
        else:
            print("Invalid response.")
    print("Testing mode: ", testing)

    while True:
        direction = input("Enter 0 to move along x slightly, 1 for backwards: ")
        if direction in ['0', '1']:
            break
        else:
            print("Invalid input. Must be 0/1.")

    if testing:
        arm = FrankaControl(debug_flag=True)

        if direction == '0':
            arm.move_relative(dx=0.05)
        elif direction == '1':
            arm.move_relative(dx=-0.05)

    else:
        dx = '0'
        dy = '0'
        dz = '0'
        if direction == '0':
            dx = '0.05'
        elif direction == '1':
            dx = '-0.05'
        print("dx: ", dx)
        print("dy: ", dy)
        print("dz: ", dz)

        program = './franka_move_to_relative'
        ip_address = '192.168.0.88'

        print("Program being run is: ", program)
        print("IP Address of robot: ", ip_address)

        command = [program, ip_address, dx, dy, dz]
        command_str = " ".join(command)

        print("Command being called: ", command_str)


def test_joints():
    """Used to test if position reporting is working from Arm.

    To use this test, add the -j or --joint-test flag to the command line.
    """
    arm = FrankaControl(debug_flag=True)
    while True:
        matrix = arm.get_joint_positions()
        print(
            "%8.6f   %8.6f   %1.0f   %1.0f   %8.6f   %8.6f   %1.0f   %1.0f   %1.0f   %1.0f   "
            "%1.0f   %1.0f   %1.0f   %1.0f   %5.3f   %1.0f" % tuple(matrix[0]))
        data = np.array(matrix)
        print(data)


def test_position():
    """Used to test if the Franka Arm is reporting the position of its end effector.

    To use this test, add the -p or --position-test flag to the command line.
    """
    arm = FrankaControl(debug_flag=True)
    pos = arm.get_end_effector_pos()
    print("End effector position:")
    print("X: ", pos[0])
    print("Y: ", pos[1])
    print("Z: ", pos[2])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Control Franka Arm with Python.')
    parser.add_argument('-m', '--motion-test', action='store_true',
                        help='run program in testing motion mode')
    parser.add_argument('-p', '--position-test', action='store_true',
                        help='run program in testing position readings mode')
    parser.add_argument('-j', '--joint-test', action='store_true',
                        help='run program in testing joint readings mode')

    args = parser.parse_args()  # Get command line args

    if args.motion_test:
        test_motion()
    elif args.position_test:
        test_position()
    elif args.joint_test:
        test_joints()
    else:
        print("Try: franka_control.py --help")
