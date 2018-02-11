****************
Project Proposal
****************

CHESS - Robotic Arm Playing Board Games
=======================================

The goal of this project is to program a robotic arm to play board games such as chess. The robot used will most likely be the FRANKA Emika robotic arm (nicknamed “Panda”) as it is more precise in positioning and grasping than DE NIRO. The outcome of the project will be demonstrated by performing a fully autonomous game of chess between a person and the robot. Perception will need to be added such as an RGB-D camera, a webcam, or a laser scanner.

You can see what the robot looks like here: http://www.imperial.ac.uk/robot-intelligence/robots/franka-emika/

The project includes:

1. **Forward and Inverse Kinematics:** To use the 7-dof kinematic model for Cartesian-to-Joint space mapping.
2. **Redundancy Resolution:** To resolve the 7-dof arm configuration while following a 6-dof hand pose trajectory.
3. **Perception:** Adding additional sensor for perceiving the environment and all objects of interest. It could be an RGB-D camera, a webcam, or a laser scanner.
4. **Object detection and recognition:** To identify the chess pieces on the board as targets for the pick-and-place operations.
5. **Chess playing software:** Finding a suitable implementation of chess playing algorithm and integrating it within the project.
6. **Motion Planning:** Using OMPL/MoveIt or other motion planning libraries to generate viable collision-free trajectories for executing the movements.
7. **Motion Control:** To tune a Cartesian impedance controller or other for fast and smooth motion of the arm.
8. **Hand control:** To control the grasping with the 2 fingers of the hand.

**Equipment:**
FRANKA Emika (Panda), RGB-D camera, laser scanner, chess board and pieces
