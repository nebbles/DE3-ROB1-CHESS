from franka_control2 import FrankaControl

arm = FrankaControl()


while True:
	llists = arm.get_pos()
	print("%8.6f   %8.6f   %1.0f   %1.0f   %8.6f   %8.6f   %1.0f   %1.0f   %1.0f   %1.0f   %1.0f   %1.0f   %1.0f   %1.0f   %5.3f   %1.0f"%tuple(llists[0]))
