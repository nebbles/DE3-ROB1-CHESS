****************************
Workstation set-up for Panda
****************************

.. note::
  This page is yet to be completed.

Generally the set up procedure follows the guide provided on the `FRANKA documentation website <https://frankaemika.github.io/docs/installation.html>`_.

Requirements for FRANKA Workstation
===================================

#. Install the latest version of Ubuntu (may need to partition hard drive; use Boot Camp on Macintosh)
#. `Install the realtime kernel patch`_
#. Once completed, ensure you are running a real-time kernel with ``uname -r``
#. Install ROS Kinetic
#. Install ``libfranka`` and ``franka_ros``
#. TBC...

Install the realtime kernel patch
=================================

https://frankaemika.github.io/docs/installation.html#setting-up-the-real-time-kernel

Install ROS Kinetic
===================

http://wiki.ros.org/kinetic/Installation/Ubuntu

Install FRANKA Libraries
========================

.. attention:: It is recommended that you **DO NOT** install from source. There are lots of problems with compiling versions which work with the current firmware version of the Panda.

Option 1: Install binaries via ``apt``
--------------------------------------

Binary packages for ``libfranka`` and ``franka_ros`` are available from the ROS repositories. After setting up `ROS Kinetic <wiki.ros.org/kinetic/Installation/Ubuntu>`_, execute::

  sudo apt install ros-kinetic-libfranka
  sudo apt install ros-kinetic-franka-ros

Note that if you use ``apt-get`` or ``apt`` to install a package on Ubuntu, you can use ``dpkg -L <packagename>`` to find where on the system the files for a particular package are installed.

Option 2: Install and compile from source (not recommended)
-----------------------------------------------------------

To install from source, you may need to use the following commands:

* ``$ git clone`` will give you the whole repository.
* ``$ git tag -l`` will list available tags.
* ``$ git checkout tags/<tag_name>`` will allow you to change repo to a specific tag, ``<tag_name>``.

Test with examples
==================

After installing ``ros-kinetic-libfranka``, you can run the examples located in ``~/git/libranka/build/examples`` such as::

  ./generate_cartesian_pose_motion <host-name>
  ./generate_cartesian_velocity_motion <host-name>

Remember, for this to work:

* The FRANKA Arm must be in movement mode (white light).
* The workstation PC must be connected to the shop floor controller by ethernet.

.. tip::
  You can confirm that the workstation computer is able to communicate with the workshop controller by pinging the IP address from the terminal: ``ping 192.168.0.88``. For more information see the :doc:`franka` page.
