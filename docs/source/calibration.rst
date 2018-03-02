***********
Calibration
***********

Introduction
============

Intro to our calibration procedure...

Procedure
=========

The procedure as follows... move in contained area, detect markers

Reference Frames
================

Overview
--------

To be able to convert camera coordinates, provided by opencv tracking tools and other methods, we need to maintain a relationship between multiple reference frames. The key relationship is that which relates the camera reference frame and the robot base reference frame used by the frankalib controller. This relationship is maintained in a 4-by-4 transformation matrix, and is constructed using the following general formula:

.. math::

  aX = b

This is modelled on the idea that we can take a coordinate in our main frame (e.g. RGB-D camera provides ``u, v, w`` coordinates) and convert it to the equivalent, corresponding coordinate in the robots reference frame (e.g. ``x, y, z``) so that the robot can move to that point on the camera's view. ``a`` represents our camera coordinate, and ``b`` represents the output of our function, that mulitplies ``a`` with our transformation matrix ``X``, which represents the same point but on the robots reference frame.

.. todo:: add image of reference frames of both robot, camera, board

Creating the transformation matrix
----------------------------------

To create the transformation matrix, we construct a set of linear equations that we want to solve using a simple least squares algorithm, commonly used in linear regression. This algorithm tries to minimise the sum of squares for each solution to the set of linear equations.

This set of linear equations is constructed using *calibration points*. These points (usually a minimum of 4) are a set of known, corresponding coordinates in both the cameras reference frame and the robots. These can be automatically sourced with a setup program, or manually. To manually get these points, the robots end effector would be moved to a point in the field of view of the camera, and the robot would report its position (``x, y, z``). The camera would then detect the robot end effector in the field of view and report the location according to its own reference frame (``u, v, w``) and so these two points are the same point (they correspond) but are in different reference frames. We collect a minimum of 4 calibration points, ideally up to 8 or 10 because this will increase the accuracy of our transformation matrix, as there may have been a small error in the values reported by the camera or robot.

.. todo:: add image of linear regression with caption

We now have our calibration equation (of *n* calibration points), and we want to solve for the unknowns in the transformation matrix, *X*.

.. math::

  \begin{bmatrix}
    u_i&v_i&w_i\\
    &\vdots&\\
    u_n&v_n&w_n
  \end{bmatrix} X =
  \begin{bmatrix}
    x_i&y_i&z_i\\
    &\vdots&\\
    x_n&y_n&z_n
  \end{bmatrix}

Where :math:`m_{ij}` is the unknown in *X*,

  .. math::

    X =\begin{bmatrix}
      m_{11}&m_{12}&m_{13}&m_{14}\\
      m_{21}&m_{22}&m_{23}&m_{24}\\
      m_{31}&m_{32}&m_{33}&m_{34}\\
      m_{41}&m_{42}&m_{43}&m_{44}
    \end{bmatrix}

In MATLab, the function for solving this equation is simply ``X = a\b``, or less commonly written as ``X = mldivide(a,b)``. `The mldivide() function`_ in MATLab is a complex one, and utilises many different possible algorithms depending on its inputs. To get the similar behaviour in Python, we use `numpy's lstsq function`_ which has similarites and differences which have been discussed `{1}`_ `{2}`_, but ultimately provides us the same functionality of returning a least square solution to the equation. We use the function as in our example belows::

  import numpy as np
  from numpy import random

  num_pts = 4

  A = random.rand(num_pts, 3)
  one = np.ones((num_pts, 1))
  A = np.column_stack([A, one])
  print("A", A)
  print("\n")

  T = random.rand(3, 4)
  xrow = np.array([0,0,0,1])
  T = np.vstack([T, xrow])
  print("T", T)
  print("\n")

  B = np.dot(A, T)
  print("B", B)
  print("\n")

  x = np.linalg.lstsq(A, B, rcond=None)[0]
  print("x", x)

.. _`The mldivide() function`: http://uk.mathworks.com/help/matlab/ref/mldivide.html
.. _`numpy's lstsq function`: https://docs.scipy.org/doc/numpy/reference/generated/numpy.linalg.lstsq.html

.. _`{1}`: https://stackoverflow.com/questions/33559946/numpy-vs-mldivide-matlab-operator
.. _`{2}`: https://stackoverflow.com/questions/33614378/how-can-i-obtain-the-same-special-solutions-to-underdetermined-linear-systems?noredirect=1&lq=1

Implementation
--------------

.. automodule:: calibration
  :members:
  :undoc-members:
