#!/usr/bin/env python
"""
Make sure that you are running ``roscore`` and ``franka_controller_sub`` before attempting to
move the robot using this class, as it publishes to ROS topics to interact with the Franka.
"""
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
import time


class FrankaRos:
    def __init__(self, log=True, ip='192.168.0.88', debug_flag=False):
        self.log = log
        self.ip_address = ip
        self.debug = debug_flag
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

    def move_to(self, x, y, z, speed):
        # rate = rospy.Rate(10) # 10hz

        self.target_coords.data = [x, y, z, speed]  # [0.1, 0.1, 0.1, 0.10]
        if self.log:
            rospy.loginfo("franka_move_to: " + str(self.target_coords.data))
        self.pub_move_to.publish(self.target_coords)
        # rate.sleep()

    def move_gripper(self, width, speed):
        self.target_gripper.data = [width, speed]  # [0.1, 0.1, 0.1, 0.10]
        if self.log:
            rospy.loginfo("franka_gripper_move: " + str(self.target_gripper.data))
        self.pub_move_grip.publish(self.target_gripper)

    def grasp(self, object_width, speed, force):
        self.target_gripper.data = [object_width, speed, force]
        if self.log:
            rospy.loginfo("franka_gripper_grasp: " + str(self.target_gripper.data))
        self.pub_grasp.publish(self.target_gripper)

    def move_gripper_example(self):
        width = 0.06  # 2.2 cm = 0 width
        speed = 0.1
        force = 1

        self.target_gripper.data = [width, speed]  # [0.1, 0.1, 0.1, 0.10]
        rospy.loginfo("franka_gripper_move: " + str(self.target_gripper.data))
        self.pub_move_grip.publish(self.target_gripper)

        time.sleep(5)

        width = 0.037  # 2.2 cm = 0 width
        self.target_gripper.data = [width, speed, force]
        rospy.loginfo("franka_gripper_grasp: " + str(self.target_gripper.data))
        self.pub_grasp.publish(self.target_gripper)

        time.sleep(5)

        width = 0.06  # 2.2 cm = 0 width
        self.target_gripper.data = [width, speed, force]
        rospy.loginfo("franka_gripper_move: " + str(self.target_gripper.data))
        self.pub_move_grip.publish(self.target_gripper)


def main():
    try:
        franka = FrankaRos()

        motion_plan = []
        resolution = 100
        for i in range(1, resolution):
            motion_plan.append((0.4+i/resolution, 0.4+i/resolution, 0.4+i/resolution, 0.100))

        # print(motion_plan)
        for x, y, z, speed in motion_plan:
            franka.move_to(x, y, z, speed)
            time.sleep(0.1)  # 10 Hz control loop

        franka.move_gripper_example()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
