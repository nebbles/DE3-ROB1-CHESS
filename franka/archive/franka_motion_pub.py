#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
import time


target_coords = Float64MultiArray()  # the three absolute target coordinates
target_coords.layout.dim.append(MultiArrayDimension())  # coordinates
target_coords.layout.dim[0].label = "coordinates" 
target_coords.layout.dim[0].size = 3

target_coords.layout.dim.append(MultiArrayDimension())  # speed
target_coords.layout.dim[1].label = "speed"
target_coords.layout.dim[1].size = 1


def franka_move_to(x, y, z, speed):
    # rate = rospy.Rate(10) # 10hz
    # target_coords
    target_coords.data = [x, y, z, speed]  # [0.1, 0.1, 0.1, 0.10]

    rospy.loginfo("franka_move_to: " + str(target_coords.data))
    # print(target_coords)
    pub.publish(target_coords)
    # rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('motion', anonymous=True)
        pub = rospy.Publisher('franka_move_to', Float64MultiArray, queue_size=1)

        motion_plan = []
        resolution = 100
        for i in range(1, resolution):         
            motion_plan.append((0.4+i/resolution, 0.4+i/resolution, 0.4+i/resolution, 0.100))

        # print(motion_plan)
        for x, y, z, speed in motion_plan: 
            franka_move_to(x, y, z, speed)
            time.sleep(0.1)  # 10 Hz control loop
        # franka_move_to(0.5, 0.5, 0.5, 0.1)
    except rospy.ROSInterruptException:
        pass
