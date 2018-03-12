from franka_control_ros import FrankaRos
import time
import numpy as np

arm = FrankaRos()

def gen_mp(s, e):
    x = e[0] - s[0]
    y = e[1] - s[1]
    z = e[2] - s[2]
    resolution = 20
    dx = x/resolution
    dy = y/resolution
    dz = z/resolution

    motion_plan = []
    for i in range(0, resolution+1):
        if i in range(0,10):
            speed = i*0.01
        else:
            speed = 0.1-(i-11)*0.01
        motion_plan.append((s[0]+i*dx, s[1]+i*dy, s[2]+i*dz, speed))

    return np.array(motion_plan)

# arm.move_gripper(0.05, 0.1)
# time.sleep(5)

# start = arm.get_position()
# end = [0.626642, 0.0284069, 0.303026]
# motion = gen_mp(s=start, e=end)

# for step in motion:
#     arm.move_to(step[0], step[1], step[2], step[3])
#     time.sleep(0.2)



# arm.move_to(0.626642, 0.0284069, 0.303026, 0.1) #before above
# time.sleep(5)
# arm.move_to(0.626642, 0.0284069, 0.113312, 0.1) #before below
# time.sleep(5)
arm.move_gripper(0.01, 0.1) #grab
time.sleep(2)
arm.move_to(0.626642, 0.0284069, 0.303026, 0.1) #before above
# time.sleep(5)
# arm.move_to(0.624103, 0.130499, 0.303026, 0.1)#new above
# time.sleep(5)
# arm.move_to(0.624103, 0.130499, 0.113312, 0.1)#new below
# time.sleep(5)
# arm.move_gripper(0.05, 0.1)
# time.sleep(5)
# arm.move_to(0.624103, 0.130499, 0.303026, 0.1)#new above


#0.626642, 0.0284069, 0.113312,

