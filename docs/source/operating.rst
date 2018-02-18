*******************
Operating the Robot
*******************

.. note::
  This page is still incomplete.

Using the provided workstation
==============================

To log into the workstation:

* Username: ``robin``
* Password: ``deniro``

.. note::
  If you do not use a provided workstation, ensure you have completed all the steps in the :doc:`workstation` page.

Starting up FRANKA
==================

Connect your workstation to the network via ethernet. Power up the FRANKA and when booted (solid yellow), type the following into the URL bar in a browser::

  http://192.168.0.88

Now run through the unlocking procedure as described in the :doc:'franka' doc.

Understanding how to control the FRANKA
=======================================

Setting Permissions
-------------------

To control the Franka Arm, Fabian and Petar have written a small collection of C++ files which can be compiled to run as executables and control the Franka Arm using libfranka.

Firstly, you need to ensure you have set the correct permissions for libfranka.

Ensure that you have run::

  source /opt/ros/kinetic/setup.bash

.. note::
  If this doesn't run, you may not have installed ROS Kinetic properly.

We then need to source ros for the root user so that libfranka has permissions to use the realtime kernel::

  sudo bash
  source /opt/ros/kinetic/setup.bash

You now press **Ctrl+D** to exit out of sudo bash (the hash sign will change back to a dollar sign). You must also check that you have the correct realtime permissions for your own user. To do this, run::

  ulimit -r

If the results is ``99`` then you have nothing more to do, is the result is ``0`` then go back and check you completed `the last section of the realtime kernel setup`_.

.. _`the last section of the realtime kernel setup`: https://frankaemika.github.io/docs/installation.html#allow-a-user-to-set-real-time-permissions-for-its-processes

Downloading the C++ Executables and Python Class
------------------------------------------------

Now that you have libfranka set up properly you can get use the C++ files provided. These resources can be found in the ``/franka`` `directory`_ of the repository. Firstly, go to your project directory in the terminal by using ``cd <project_dir>``. If you have already downloaded the files before and are replacing them with an up-to-date version, run ``rm -rf franka/`` first. To download the necessary folder, run::

  svn export https://github.com/nebbles/DE3-ROB1-CHESS/trunk/franka

.. _`directory`: https://github.com/nebbles/DE3-ROB1-CHESS/tree/master/franka

Once this directory is downloaded into your project directory, you need to change directory and then make the binaries executable::

  cd franka/
  chmod a+x franka*
  chmod a-x *.cpp

.. warning::
  This next command will move the FRANKA. **Make sure you have someone in charge of the external activation device (push button)**.

These binaries can now be used from the command line to control the Arm::

  ./franka_move_to_relative <ip_address> <delta_X> <delta_Y> <delta_Z>

Alternatively, you can control the Arm using the easy custom Python class ``Caller`` (see below).

Python-Franka API with ``franka_control.py``
--------------------------------------------

The Python-FRANKA module (``franka_control.py``) is designed to allow easy access to the C++ controller programs provided by Petar. The provided Python module is structured as follows.

.. automodule:: franka.franka_control
  :members:
  :undoc-members:

Example
~~~~~~~

To use the ``FrankaControl`` class in your own Python script would look something like this:

.. code-block:: python

   from franka.franka_control import FrankaControl

   arm = FrankaControl(debug_flag=True)
   # we set the flag true to get prints to the console about what Caller is doing

   arm.move_relative(dx=0.1, dy=0.0, dz=-0.3)
   # we tell teh arm to move down by 30cm and along x away from base by 10cm

.. note::
  This example code assumes you are following the general project structure guide. See below for more information. The code above would be called from a main script such as ``run.py``.

General Structure of Project
----------------------------

The structure of the project is important to ensure that importing between modules works properly and also seperates externally maintained code from your own. An example of a project tree is::

  .
  ├── LICENSE.txt
  ├── README.md
  ├── run.py
  ├── __init__.py
  ├── docs
  │   ├── Makefile
  │   ├── build
  │   ├── make.bat
  │   └── source
  ├── franka
  │   ├── __init__.py
  │   ├── franka_control.py
  │   ├── franka_get_current_position
  │   ├── franka_get_current_position.cpp
  │   ├── franka_move_to_absolute
  │   ├── franka_move_to_absolute.cpp
  │   ├── franka_move_to_relative
  │   └── franka_move_to_relative.cpp
  ├── my_modules
  │   ├── module1.py
  │   ├── module2.py
  │   ├── module3.py
  │   ├── __init__.py
  └── test_script.py


.. Planned method
.. --------------
..
.. The image below describes how we plan to control the Arm using Python. To be able to write a successful Python program, we must first understand how ROS works: how to publish and listen on topics.
..
.. .. figure:: _static/franka_programming_interface.png
..     :align: center
..     :figclass: align-center
..
..     Interfacing Python with FRANKA.


Getting Started with ROS
========================

.. note::
  Currently we are not using ROS to control the FRANKA Robot. So this section is only necessary to get practise in setting up a workspace. We will likely be using this in the future.

#. In your home directory, ensure you have set up a `complete catkin workspace`_.
#. Within that workspace, `create a catkin package`_.
#. TBC...

.. _`complete catkin workspace`: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
.. _`create a catkin package`: http://wiki.ros.org/ROS/Tutorials/CreatingPackage

Additional Resources
====================

https://frankaemika.github.io/docs/getting_started.html#operating-the-robot
