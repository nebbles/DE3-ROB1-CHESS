******
Motion
******

Description
===========

This part of the project is responsible for providing the FRANKA arm with instructions on when and where to move. It takes in the algebraic notation (AN) of the start and end locations of the desired move, and then generates a motion plan to be sent to FRANKA for execution.


Callibration
============

When the TODO:XXX function is instantiated, the callibration procedure is executed.
The 4 inner corner coordinates gathered by the perception module are stored and these are used as the base information. 

.. figure:: _static/callibration_positions.png
    :align: center
    :figwidth: 20 em
    :figclass: align-center 

4 vectors are calculated from this information, along the x and y axis of the board:

- x axis 1: H8 to A8 
- x axis 2: H1 to A1
- y axis 1: H8 to H1
- y axis 2: A8 to A1

.. figure:: _static/reference_frame_diagram.png
    :align: center
    :figwidth: 20 em
    :figclass: align-center

The x and y vectors are averaged respectively to give overall x and y directional vectors.
These are then divided by 6 to give x and y unit vetors equivalent to a single board square.
The H8 coordinate is stored and used as the starting point for all future location calculations.

The maximum z value of the board is extracted from the perception coordinates. This is then used to hardcode a hover height, rest position and deadzone location as global variables. 

The coordinates of each square on the board are stored using 2 dictionaries, one for the numbers and one for the letters in AN. These locations are found by multiplying the unit vectors appropriately. For example, ``f`` is found by adding 2.5 times ``x_unit_vector`` to the H8 coordinate. 

Derived positions:

.. figure:: _static/derived_positions.png
    :align: center
    :figwidth: 20 em
    :figclass: align-center 

The ``x,y,z`` coordinates to pick up each piece were also stored in a global dictionary. Hardcoded width values were used for the gripping width, whilst the ``board_z_max`` was used to derive the correct height at which to pick up each piece.

Iterations
==========

**Manual Callibration**

This process was used to callibrate the robot before the perception method was implemented. Instructions were sent to the console, informing the user to move the FRANKA end point to A1, A8, H8, H1, the deadzone, the rest position and the desired hover height. After each instruction is sent, the real world coordinates of FRANKA were collected and stored.


Grippers
========

Specialised grippers were designed to pick up the chess pieces. 

**Iteration 1:**

A compliant design was used which allowed the grippers themselves to deform slightly when each piece was gripped. This allowed the uneven bumps of some pieces to be successfully gripped. They were 3D printed on the ProJet printer and then material from disposable silicone gloves was used to create the pads.

.. figure:: _static/gripper_1.png
    :align: center
    :figwidth: 20 em
    :figclass: align-center 

**Iteration 2:**

Small updates were made to the original design foe the final versions. A taper angle was added to accomodate for pieces that are wider at the top than lower down. The width of the compressible part was increased to improve grip. 

.. figure:: _static/gripper_2.png
    :align: center
    :figwidth: 20 em
    :figclass: align-center 

Design
======

.. tip::
   En passant moves have been disabled for this project, so the location of the killed piece will always be the same as the goal location.	

Firstly, the start and goal positions of the piece are converted from algebraic notation (AN) to ``x, y, z`` coordinates in FRANKA's reference frame. The AN is extracted from the output of the game engine, along with the piece type (king/queen etc). The conversion from AN is done by finding the x and y vectors from H8 to the selected square. Letter and number dictionaries of vectors are indexed through to find the necessary vectors. These are then summed, giving the position vector of the square. For example, square F6 would be found by summing the F vector and the 6 vector, as shown below. The ``z`` coordinate is selected based on what the piece is, giving the best gripping height for that piece.

.. figure:: _static/vectors.png
    :figwidth: 20 em
    :align: center
    :figclass: align-center

.. note::
    These dictionaries are all created when the TODO:XXX function is instantiated.

The intermediate positions are then found; these are dependant on whether or not a piece has been killed, as shown below. The locations have been selected to allow FRANKA to follow a repeatable, direct path no matter what coordinates are parsed. The hover positions have ``x, y`` coordinates based on which pieces are being picked up while the ``z`` coordinate is hardcoded. The deadzone and rest positions are also hardcoded and created in the initialisation of the function. 

.. figure:: _static/paths.png
    :align: center
    :figclass: align-center

.. tip::
	The trajectory is vertically straight for all motions where a piece is gripped or ungripped on the board. This prevents the arm from colliding with any other pieces. The length of this section is determined by the hover height.

The positions are outputted in pairs of start and end locations, with every intermediate position represented. The start position is always set to the current position of FRANKA, rather than the end position of the previous command. This would account for any errors in FRANKA's motion. 

.. literalinclude:: ../../motion/Trajectory.py
   :lines: 66-80

THIS SHOULD BE THE CODE THAT SHOWS THE PAIRS OF POSITIONS ^^^

.. note::
	An additional move taking FRANKA from it's current position to it's rest position is always included in case the arm is not already in it's rest location.


**Velocity Profile**

BEN TO DO
Define that a **series** is made up of paths that surround gripping actions
Should i write about sleep?

**Executing the trajectory**

Before any trajectory is executed, the gripper is moved to it's ungripped position. This dimension is again hardcoded. Doing this at the beginning of the algorithm prevents any collisions between the end effector and the peices.

A single path is considered as a movement between 2 locations that involve a gripping action, for example start and goal, goal and deadzone or goal and rest. Only one path is executed at a time. 

The path is extracted from the series generated by the XXX function. A trapezoidal velocity profile is then applied as described above. This path is sent to the FRANKA arm for execution. The current position of the end effector is updated. 

It is then determined whether or not a gripping action should be executed, based on how many paths have already been run. This is possible as there are only 2 different series that could be executed (dead and not dead). If a gripping action is required, the desired gripping width is found using a global dictionary. The gripping action is executed by FRANKA. 

This exectution process is repeated as many times as there are paths in the series, completing a single chess move.


.. tip::
   En passant moves have been disabled for this project, so the location of the killed piece will always be the same as the goal location.	

Once this information has been collected, the following algorithm can be implemented:


Iterations
==========

**Smoothing the trajectory**

3 different methods were attempted to try and smooth the path to give a more natural movement.

1)	Interpolation

The first method attempted was interpolation. This generated knot points along a given path and joined them up using a spline.

.. figure:: _static/interpolation.png
    :figwidth: 20 em
    :align: center
    :figclass: align-center

.. tip::
	 Interpolation is a method of constructing new data points within the range of a discrete set of known data points. Spline interpolation uses low-degree polynomials in each of the intervals, and chooses the polynomial pieces such that they fit smoothly together. The resulting function is called a spline. The interpolant is easier to evaluate than the high-degree polynomials used in polynomial interpolation. **(TODO: REF wiki)**

The B-spline representation of the 3-dimensional trajectory was found using a ``scipy`` function. This function takes in a list of coordinates that represent a trajectory as well as a smoothing factor. It returns the knot locations, the B-spline coefficients and the degree of the spline. 

These outputs were then given to another ``scipy`` function that evaluates the value of the smoothing polynomial and it's derivatives. It returns the coordinates of the knots and a list of coordinates making up the smoothed path.

The problem with this method was a lack of control over the path and difficultly in finding a consistently reliable smoothing factor. 

2)	Tortoise and Hare

A discretised list of points was duplicated ans shifted, such that one was several steps ahead of the other. The corresponding points in each list were then averaged, producing a third list of points. The result of this was a chamfered rather than smoothed corner.

.. figure:: _static/chamfer.png
    :figwidth: 20 em
    :align: center
    :figclass: align-center

3)	Repeated Tortoise and Hare

To correct this problem, the same algrithm was run multiple tmies, chamfering the chamfer. This created a smooth path:

.. figure:: _static/chamfered_chamfer.png
    :figwidth: 10 em
    :align: center
    :figclass: align-center

The problem here is that the distance between the points on the chamfered edges is smaller than the distance between points on unaffected edges. This did not work with the velocity profile algorithm.

Limitations
===========

Feedback
Lack of smoothness


Implementation
==============

**Example usage**::

  Put some code stuff here


**Documentation**:

.. automodule:: motion.Trajectory
  :members:
  :undoc-members: