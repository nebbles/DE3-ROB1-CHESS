#!/usr/bin/env python



import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
import time



# gripper
target_gripper = Float64MultiArray()  # gripper stuff
target_gripper.layout.dim.append(MultiArrayDimension())  # gripper
target_gripper.layout.dim[0].label = "gripper" 
target_gripper.layout.dim[0].size = 3  # width, speed, force


if __name__ == '__main__':
    try:
        width = 0.06 # 2.2 cm = 0 width
        speed = 0.1  
        force = 1 
        rospy.init_node('gripper_node', anonymous=True)
        pub_grasp = rospy.Publisher('franka_gripper_grasp', Float64MultiArray, queue_size=0)
        pub_move = rospy.Publisher('franka_gripper_move', Float64MultiArray, queue_size=0)
        time.sleep(0.5)  # required, otherwise the first message is not published

        #rate = rospy.Rate(10) # 1hz
        # target_coords
        target_gripper.data = [width, speed]  # [0.1, 0.1, 0.1, 0.10]

        rospy.loginfo("franka_gripper_move: " + str(target_gripper.data))
        # print(target_coords)

#        while(True): 
        pub_move.publish(target_gripper)

        time.sleep(5)

        width = 0.037 # 2.2 cm = 0 width

                # target_coords
        target_gripper.data = [width, speed, force]  # [0.1, 0.1, 0.1, 0.10]

        rospy.loginfo("franka_gripper_grasp: " + str(target_gripper.data))
        # print(target_coords)

#        while(True): 
        pub_grasp.publish(target_gripper)

        time.sleep(5)

        width = 0.06 # 2.2 cm = 0 width

                # target_coords
        target_gripper.data = [width, speed, force]  # [0.1, 0.1, 0.1, 0.10]

        rospy.loginfo("franka_gripper_move: " + str(target_gripper.data))
        # print(target_coords)

#        while(True): 
        pub_move.publish(target_gripper)

 #            rate.sleep()

        # print("before spin")

        # rospy.spin()

        # motion_plan = []
        # resolution = 100
        # for i in range(1, resolution):         
        #     motion_plan.append((0.4+i/resolution, 0.4+i/resolution, 0.4+i/resolution, 0.100))

        # # print(motion_plan)
        # for x, y, z, speed in motion_plan: 
        #     franka_move_to(x, y, z, speed)
        #     time.sleep(0.1)  # 10 Hz control loop
        # franka_move_to(0.5, 0.5, 0.5, 0.1)
    except rospy.ROSInterruptException:
        pass