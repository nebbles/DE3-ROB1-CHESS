import subprocess
import os


class Caller:
    """
    Class which contains methods to control the Franka Panda using supporting C++ binaries.
    """
    def __init__(self, ip='192.168.0.88', debug_flag=False):
        self.ip_address = ip
        self.debug = debug_flag

    def move_relative(self, dx: float=0.0, dy: float=0.0, dz: float=0.0):
        """
        Executes Franka C++ binary which moves the arm relative
        to its current position according to the input arguments.

        Returns the return_code of the C++ program.
        """
        try:
            dx, dy, dz = float(dx), float(dy), float(dz)
        except ValueError:
            print("Arguments are invalid: must be floats")
            return

        dx, dy, dz = str(dx), str(dy), str(dz)

        path = os.path.dirname(os.path.realpath(__file__))  # gets working dir of this file
        program = './franka_move_to_relative'  # set executable to be used
        command = [program, self.ip_address, dx, dy, dz]
        command_str = " ".join(command)

        if self.debug:
            print("Working directory: ", path)
            print("Program: ", program)
            print("IP Address of robot: ", self.ip_address)
            print("dx: ", dx)
            print("dy: ", dy)
            print("dz: ", dz)
            print("Command being called: ", command_str)
            print("Running FRANKA code...")

        return_code = subprocess.call(command, cwd=path)

        if return_code == 0:
            if self.debug:
                print("No problems")
        else:
            print("Python has registered a problem with ", program)

        return return_code

    def move_absolute(self, coordinates: list):
        if len(coordinates) > 3:
            raise ValueError("Invalid coordinates. There can only be three dimensions.")
        x, y, z = coordinates[0], coordinates[1], coordinates[2]

        # TODO: implement safety check for target coordinates

        path = os.path.dirname(os.path.realpath(__file__))  # gets working dir of this file
        program = './franka_move_to_absolute'
        command = [program, self.ip_address, x, y, z]
        command_str = " ".join(command)

        if self.debug:
            print("Program: ", program)
            print("IP Address of robot: ", self.ip_address)
            print("Go to x: ", x)
            print("Go to y: ", y)
            print("Go to z: ", z)
            print("Command being called: ", command_str)
            print("Running FRANKA code...")

        return_code = subprocess.call(command, cwd=path)

        if return_code == 0:
            if self.debug:
                print("No problems")
        else:
            print("Python has registered a problem with ", program)


def main():
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
        arm = Caller(debug_flag=True)

        if direction == '0':
            arm.move_relative(dx=0.05)
        elif direction == '1':
            arm.move_relative(dx=-0.05)

    else:
        dx = '0'
        dy = '0'
        dz = '0'
        if direction == '0':
            dx = 0.05
        elif direction == '1':
            dx = -0.05
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


if __name__ == '__main__':
    main()
