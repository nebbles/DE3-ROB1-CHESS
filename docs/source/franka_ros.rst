************
Franka & ROS
************

Starting ROS to control Franka
==============================

Using a single workstation
--------------------------

To use ROS to control the Franka arm from one workstation, you need to have the master node running for ROS. To do this, it can be done in a seperate terminal window (run ``roscore`` in the command line), or from Python with::

  import subprocess
  roscore = subprocess.Popen('roscore')
  time.sleep(1)  # wait a bit to be sure the roscore is really launched

From this point, you can now run the subscriber node.

Networking with other workstations
----------------------------------

Instead of running the master node and subscriber nodes on your own workstation, these can be running on the main workstation in the lab instead. This means that libfranka won't need to be installed on your specific workstation.

To communicate over the lab network you need to change two main ROS variables. Firstly you need to find the IP address of your computer when connected to the lab network (via ethernet). To do this you can use ``ifconfig`` in a terminal window to give you your ``<ip_address_of_pc>``.

You then need to run the following two commands in your terminal window (substitute in you IP address)::

  export ROS_MASTER_URI=http://192.168.0.77:11311
  export ROS_IP=<ip_address_of_pc>

As you will see, this is connecting you to the static IP address of the main Franka workstation, ``192.168.0.77``. In order for you to continue with running a Python publisher, you need to ensure that roscore and the subscriber is running on the main workstation.

.. note:: That this configuration of assigning IP addresses to ROS_MASTER_URI and ROS_IP is non-permanent, and is only active for the terminal window you are working in. This has to be repeated for every window you use to run rospy. Alternatively you can add these commands to your bashrc.

Running the subscriber
======================

Once roscore is running, the subsciber has to run in the background to convert the ros messages into Franka commands and execute. For this, execute the subscriber binary file.

.. warning:: This file is currently only compiled to run with libfranka 0.1.0 (you can check your current libfranka version with ``rosversion libfranka`` in the terminal).

.. tip:: Sometimes there is an '*error with no active exception*' thrown by this executable. This can sometimes be solved by simply manually moving the arm using the buttons.

Using the publisher
===================

.. todo:: Add the information on motion and gripper publishing.

Using the franka_ros library
============================

The Franka ROS packages are intiated using the launch xml files. To do this you need to adjust the default IP address in these launch files::

  roscd franka_visualization
  cd launch/
  nano franka_visualization.launch

Now change the value for ``name=robot_ip`` from ``default=robot.franka.de`` to ``default=192.168.0.88``. You can then launch the package with::

  roslaunch franka_visualization franka_visualization.launch

.. note:: Swap out the 'visualization' term for any other franka_ros package.s

Getting Started with ROS
========================

To 'get started' with learning ros, you may find doing the following helps you to understand ROS better:

#. In your home directory, ensure you have set up a `complete catkin workspace`_.
#. Within that workspace, `create a catkin package`_.

.. _`complete catkin workspace`: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
.. _`create a catkin package`: http://wiki.ros.org/ROS/Tutorials/CreatingPackage
