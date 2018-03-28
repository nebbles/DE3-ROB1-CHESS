.. DE3-ROB1-CHESS documentation master file, created by
   sphinx-quickstart on Sat Feb  3 21:22:24 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

**********************************
DE3-ROB1 CHESS Group Documentation
**********************************

The goal of this project was to create a fully automated chess playing robot by applying code to a FRANKA Panda arm. The project was written in Python and ROS was used to interface with FRANKA.

Open source code was used for the game engine, giving the robot the ability to calculate the best next move. Open Computer Vision libraries were used to visualise the board. This allowed the robot to automatically know what the opponent's last move was. A virtual chess clock allowed the user to finish their move, alerting FRANKA to calculate the next move. Motion planning was used to create a path for FRANKA to follow and control the grippers. A smooth velocity profile was applied, making the motion more smooth and natural. A manual calibration process was used to send accurate and precise coordinates to FRANKA.

Future improvements to the project could include a final implementation of automated calibration as well as a smoother trajectory. Overall, the main aims of this project were successfully achieved. A complete game of chess could be played between human and robot.

About
=====

This is the documentation for the group Chess Project for the Robotics 1 module in Design Engineering, Imperial College London, 2018.

The authors: Anna Bernbaum, Ben Greenberg, Josephine Latreille, Sanish Mistry, Leah Pattison, Paolo Ruegg, Sylvia Zhang.

For enquiries on this documentation or source code, please contact bsg115@ic.ac.uk

For help with out perception code, please contact either leah.pattison15@imperial.ac.uk or paolo.ruegg15@imperial.ac.uk

Popular Links
=============

* `Using Python to control Franka (without ROS)`_.
* `Using Python to control Franka with ROS topics`_.
* `Converting points between reference frames`_.

.. _`Using Python to control Franka (without ROS)`: operating.html
.. _`Using Python to control Franka with ROS topics`: franka_ros.html
.. _`Converting points between reference frames`: calibration.html

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
