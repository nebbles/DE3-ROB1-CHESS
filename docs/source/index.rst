**********************************
DE3-ROB1 CHESS Group Documentation
**********************************

About
=====

This is the documentation for the group CHESS Project for the Robotics 1 module in Design Engineering, Imperial College London, in March 2018.

The project is hosted on GitHub: https://github.com/nebbles/DE3-ROB1-CHESS.

The Authors
-----------

* Anna Bernbaum
* Benedict Greenberg
* Josephine Latreille
* Sanish Mistry
* Leah Pattison
* Paolo Ruegg
* Sylvia Zhang

For enquiries on this documentation or source code, please contact benedict.greenberg15@imperial.ac.uk

For help with our perception code, please contact either leah.pattison15@imperial.ac.uk or paolo.ruegg15@imperial.ac.uk

Summary
=======

The goal of this project was to create a fully automated chess playing robot by applying code to a FRANKA Panda arm. The project was written in Python and ROS was used to interface with FRANKA.

Open source code is used for the chess A.I., giving the robot the ability to calculate the best next move. Open Computer Vision libraries is used to process images of the board. This allows the robot to understand what the opponent's last move was. A virtual chess clock allows the user to finish their move, alerting FRANKA to calculate the next move. Custom built motion planning is used to create a path for FRANKA to follow and control the grippers. A smooth trapeziem velocity profile was applied, making the motion controlled, natural, and reliable. A manual calibration process was used to send accurate and precise coordinates to FRANKA.

Future improvements to the project could include a final implementation of automated calibration as well as a smoother, more continous trajectory. Additionally, the final implementation of the chess clock is yet to be completed. Overall, the main aims of this project were successfully achieved. A game of chess could be played between human and robot.

Popular Links
=============

* `Using Python to control Franka (without ROS)`_.
* `Using Python to control Franka with ROS topics`_.
* `Converting points between reference frames`_.

.. _`Using Python to control Franka (without ROS)`: operating.html
.. _`Using Python to control Franka with ROS topics`: franka_ros.html
.. _`Converting points between reference frames`: calibration.html

Using Test Scripts
==================

Throughout our project we used test scripts. These can be seen in the ``tests`` folder. To run these tests properly (e.g. ``test_camera.py``) you should type the following into the terminal::

  cd DE3-ROB1-CHESS/
  python -m tests.test_camera.py

This is to ensure relative imports work properly, as every import is relative to the project level directory.

Contents
========

.. toctree::
   :maxdepth: 2
   :caption: Working with FRANKA Emika

   franka
   workstation
   operating

.. toctree::
   :maxdepth: 2
   :caption: Project Development

   camera
   calibration
   perception
   chess-engine
   motion
   controller

.. toctree::
   :maxdepth: 2
   :caption: Project Management

   project-proposal
   project-plan

.. toctree::
   :maxdepth: 2
   :caption: Appendix

   resources
   ground-rules

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
