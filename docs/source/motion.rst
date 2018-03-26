******
Motion
******

Description
===========

This part of the project is responsible for controlling the FRANKA arm. It takes in the algebraic notation (AN) produced by the game engine and converts it into real world coordinates. It then forms a path to the desired locations and executes this, as well as the gripping and ungripping movements.


Design
======

Before forming a path, the desired locations to travel from and to are required. These are the Key Locations:

	- The rest position of FRANKA
	- The current location of the piece to be moved
	- The desired location of the piece to be moved

If a piece has been killed, this extends to include:

	- The location of the piece to be killed
	- The location of the dead zone

.. tip::
   En passant moves have been disabled for this project, so the location of the killed piece will always be the same as the goal location.	

Once this information has been collected, the following algorithm can be implemented:

**1)  Resolution**

A resolution is selected to determine how many points there should be on the outputted trajectory. This is done by specifying how many millimetres there are between every point.

**2)  Logic to extract information from output of game engine**

The output of the game engine is dissected into 2 different pieces of information:
	- Whether or not a piece has died
	- The algebraic notation of the start, goal (and died) pieces

.. literalinclude:: ../../motion/Trajectory.py
   :lines: 66-80

**3)  Real world conversion**

The real world coordinates of the start, goal (and dead) pieces in the FRANKA frame are found by converting from AN to ``x, y, z``. This is done by assigning the A1, A8 and H8 squares to the real life coordinates collected from FRANKA. By finding the width of each square from this information, the real world coordinates (in the FRANKA frame) of each AN location can be calculated. The ``z`` coordinate was set to 0 for every AN location.

.. literalinclude:: ../../motion/Trajectory.py
   :lines: 92-114

**4)  Intermediate poses**

The coordinates in the FRANKA frame of some key intermediate positions, as shown below are determined. These have been selected to allow FRANKA to follow a repeatable, direct path no matter what coordinates are parsed.

.. tip::
	The trajectory is vertically straight for all motions where a piece is gripped or ungripped. This prevents the arm from colliding with any other pieces on the board. The length of this section is determined by the hover height.

.. figure:: _static/paths.png
    :align: center
    :figclass: align-center

**5)  Straight line path**

A straight line trajectory is generated, joining up the start, intermediate and end locations. The outputted path depends on whether a piece has died or not; these 2 options are shown above. The straight lines are simply created by generating a number of equally spaced points between a start and end coordinate. 

**6)  Smooth the trajectory**

The outputted line trajectory is smoothed to give a more natural path. This is done using interpolation to find knot points along the path and join them up using a spline. 

.. tip::
	 Interpolation is a method of constructing new data points within the range of a discrete set of known data points. Spline interpolation uses low-degree polynomials in each of the intervals, and chooses the polynomial pieces such that they fit smoothly together. The resulting function is called a spline. The interpolant is easier to evaluate than the high-degree polynomials used in polynomial interpolation. **(REF wiki)**

The B-spline representation of the 3-dimensional trajectory is found using a ``scipy`` function. This function takes in a list of coordinates that represent a trajectory as well as a smoothing factor. It returns the knot locations, the B-spline coefficients and the degree of the spline. 

These outputs are then given to another ``scipy`` function that evaluates the value of the smoothing polynomial and it's derivatives. It returns the coordinates of the knots and a list of coordinates making up the smoothed path.

**7)  Output**

The completed, smooth trajectory is outputted and sent to the FRANKA control code.


Iterations
==========

**Manual Callibration**

This process was used to callibrate the robot before the perception method was implemented. Instructions were sent to the console, informing the user to move the FRANKA end point to A1, A8, H8, H1, the deadzone, the rest position and the desired hover height. After each instruction is sent, the real world coordinates of FRANKA were collected and stored.

.. figure:: _static/callibration_positions.png
    :align: center
    :figclass: align-center 

Limitations
===========

Feedback
Gripping

Implementation
==============

**Example usage**::

  Put some code stuff here


**Documentation**:

.. automodule:: motion.Trajectory
  :members:
  :undoc-members: