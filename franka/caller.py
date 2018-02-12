import subprocess
import time


class Caller:
    def __init__(self, ip='192.168.0.88', debug_flag=False):
        self.ip_address = ip
        self.debug = debug_flag

    def move_relative(self, dx=0, dy=0, dz=0):
        program = './franka_move_to_relative'
        command = [program, self.ip_address, dx, dy, dz]
        command_str = " ".join(command)

        if self.debug:
            print("Program: ", program)
            print("IP Address of robot: ", self.ip_address)
            print("dx: ", dx)
            print("dy: ", dy)
            print("dz: ", dz)
            print("Command being called: ", command_str)
            print("Running FRANKA code...")

        return_code = subprocess.call(command)

        if return_code == 0:
            if self.debug:
                print("No problems")
        else:
            print("Python has registered a problem with the subprocess.")

    def move_absolute(self, x, y, z):
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

        return_code = subprocess.call(command)

        if return_code == 0:
            if self.debug:
                print("No problems")
        else:
            print("Python has registered a problem with the subprocess.")


def main():
    while True:
        testing = input("Is this program being tested [N/y]: ")
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
        direction = input("Type 0 for forward, 1 for back: ")
        if direction in ['0', '1']:
            break
        else:
            print("direction was not understood")

    print("Direction is: ", direction)

    dx = '0'

    if direction == '0':
        dx = '0.05'
    elif direction == '1':
        dx = '-0.05'

    dy = '0'
    dz = '0'

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

    if testing:
        print("Running the actual FRANKA code in 3 seconds!")
        time.sleep(3)
        print("Running it now!")
        returncode = subprocess.call(command)
        print("Python reads the return code as: ", returncode)
        if returncode != 0:
            print("Python has registered a problem with the subprocess.")


if __name__ == '__main__':
    main()
