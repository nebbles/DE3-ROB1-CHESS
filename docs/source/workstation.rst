****************************
Workstation set-up for Panda
****************************

Generally the set up procedure follows the guide provided on the `FRANKA documentation website <https://frankaemika.github.io/docs/installation.html>`_.

Requirements for FRANKA Workstation
===================================

#. `Install Ubuntu`_ (you may need to partition hard drive)
#. `Install the realtime kernel patch`_
#. `Install the realtime kernel patch`_
#. `Install ROS Kinetic`_
#. `Install FRANKA Libraries`_
#. `Install other libraries`_

Install Ubuntu
==============

On Macintosh
------------

It is recommended you follow the following guide to install Ubuntu alongside your current operating systems on Mac:

https://apple.stackexchange.com/questions/257166/installing-ubuntu-on-mac-with-macos-and-windows-already-installed

On Windows PC
-------------

To install Ubuntu alongside Windows, you must first shrink you current partitions down so there is space to create a new partition. To do this, follow `this dual boot guide`_.

If you are unable to shrink a partition with free space available, you need to defrag you partition. Unfortunately this may, or may not work just using Windows' built in application. If it doesn't work, use a third party tool, like the free `AOMEI Partition Assistant`_ to shrink your drive with space.

.. _`this dual boot guide`: https://www.howtogeek.com/214571/how-to-dual-boot-linux-on-your-pc/
.. _`AOMEI Partition Assistant`: https://www.aomeitech.com/aomei-partition-assistant.html


After you have shrunk your drive and you have a suitable amount of space for Linux (50-100GB ideally) you need to check how many partitions you currently have. If you already have 4 partitions, e.g.:

* System (Windows)
* Recovery
* HP_RECOVERY
* HP_TOOLS

Then you should delete the HP_RECOVERY partition using the disk management tool in Windows. This is because Master Boot Record (the partition map that Windows traditionally uses) only allows for a maximum of 4 partitions. (Also see: https://support.hp.com/gb-en/document/c00810279).

.. note::
  You should check if your partition map is MBR (Master Boot Record) or GUID/GPT. If it is GUID/GPT then you do not need to delete any partitions in order to create a new one.

You should download the latest Ubuntu LTS version from the official website, then use a tool like RUFUS to create a bootable USB. Follow this guide to create a bootable USB for Linux: https://www.howtogeek.com/howto/linux/create-a-bootable-ubuntu-usb-flash-drive-the-easy-way/

When you have created your bootable USB, restart your computer and (if on an HP computer) continually tap the F9 key. This will lead you to the boot menu allowing you to select the USB that you just made to boot Linux.

You can now continue with `this guide <https://www.howtogeek.com/214571/how-to-dual-boot-linux-on-your-pc/>`_ as before to install Linux. Once it is completely installed, you can move to the next section.

Install the realtime kernel patch
=================================

https://frankaemika.github.io/docs/installation.html#setting-up-the-real-time-kernel

Check you are running a real-time kernel with ``uname -r``.

If you are not,

* Restart the computer
* Select *Advanced options for Ubuntu*
* Select the kernel with ``rt`` in the name.

Install ROS Kinetic
===================

Follow the official installation guide to get ROS Kinetic at the following URL:

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

Option 2: Install and compile from source (**not recommended**)
---------------------------------------------------------------

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

Install other libraries
=======================

It is recommended that you install OMPL:

http://ompl.kavrakilab.org/installation.html

.. warning::
  The installation of OMPL takes several hours.
