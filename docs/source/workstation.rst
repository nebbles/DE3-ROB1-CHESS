*****************************
Workstation set-up for Franka
*****************************

Generally the set up procedure follows the guide provided on the `FRANKA documentation website <https://frankaemika.github.io/docs/installation.html>`_.

Requirements for FRANKA Workstation
===================================

#. `Install Ubuntu`_ (you may need to partition hard drive)
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

.. attention:: It is recommended that you **DO NOT** install from source. There are lots of problems with compiling versions which work with the current firmware version of the Franka Arm.

Option 1: Install binaries via ``apt``
--------------------------------------

.. warning:: Franka Emika have updated the version of libfranka distributed by ROS from 0.1.0 to 0.2.0 which means it no longer supports the firmware version used in the lab. For more information on software versions, see :ref:`franka-emika-software`. Unless the Franka's firmware has been updated, you should now use `Option 2: Install and compile from source`_.

Binary packages for ``libfranka`` and ``franka_ros`` are available from the ROS repositories. After setting up `ROS Kinetic <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_, execute::

  sudo apt install ros-kinetic-libfranka
  sudo apt install ros-kinetic-franka-ros

Note that if you use ``apt-get`` or ``apt`` to install a package on Ubuntu, you can use ``dpkg -L <packagename>`` to find where on the system the files for a particular package are installed.

Option 2: Install and compile from source
-----------------------------------------

*The following guide is based on a guide from Franka Emika* (`link <https://frankaemika.github.io/docs/installation.html>`__).

Before building from source, please uninstall existing installations of ``libfranka`` and
``franka_ros`` to avoid conflicts::

    sudo apt remove "*libfranka*"

To build ``libfranka``, install the following dependencies from Ubuntu's package manager::

    sudo apt install build-essential cmake git libpoco-dev libeigen3-dev

Then, download the source code by cloning ``libfranka`` from `GitHub <https://github.com/frankaemika/libfranka>`__:

.. code-block:: shell

    cd ~
    git clone --recursive https://github.com/frankaemika/libfranka
    cd libfranka

By default, this will check out the newest release of ``libfranka``. If you want to build a particular version of ``libfranka`` instead, check out the corresponding Git tag. At the time of writing the firmware of the Franka means we need version ``0.1.0`` so we do::

    git checkout tags/0.1.0
    git submodule update

.. tip:: Use ``git tag -l`` will list available tags.

In the source directory, create a build directory and run CMake:

.. code-block:: shell

    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    cmake --build .

You now need to add the path of this library to you system path so that libfranka can be found at runtime. To do this, when in the ``build/`` directory:

.. code-block:: shell

    pwd

This returns a path such as ``/home/<username>/libfranka/build``. We then add this to the **end** of our ``~/.bashrc`` file, such an example is:

.. code-block:: shell

    nano ~/.bashrc

    # add the following line to the end of the file
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/home/<username>/libfranka/build"
    # now save and exit the file

    source ~/.bashrc

.. warning:: Remember to put your own path to the build directory of ``libfranka`` instead of ``/home/<username>/libfranka/build``.

Installing ``libfranka`` is now complete. If you now need to build the ros packages, you should use the guide `found here`_.

.. _found here: https://frankaemika.github.io/docs/installation.html#building-the-ros-packages

.. _setting-permissions:

Using the franka_ros library
============================

*This is only applicable is you installed* ``franka_ros`` *with* ``apt``.

The Franka ROS packages are intiated using the launch xml files. To do this you need to adjust the default IP address in these launch files::

  roscd franka_visualization
  cd launch/
  nano franka_visualization.launch

Now change the value for ``name=robot_ip`` from ``default=robot.franka.de`` to ``default=192.168.0.88``. You can then launch the package with::

  roslaunch franka_visualization franka_visualization.launch

You can swap out the 'visualization' term for any other franka_ros package.

Setting Permissions
===================

You need to ensure you have set the correct permissions for libfranka. Run::

  source /opt/ros/kinetic/setup.bash

.. hint::
  If this doesn't run, you may not have installed ROS Kinetic properly. Check `ROS Kinetic install here <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_.

We then need to source ros for the root user so that libfranka has permissions to use the realtime kernel::

  sudo bash
  source /opt/ros/kinetic/setup.bash

You now press **Ctrl+D** to exit out of sudo bash (the hash sign will change back to a dollar sign). You must also check that you have the correct realtime permissions for your own user. To do this, run::

  ulimit -r

If the results is ``99`` then you have nothing more to do, is the result is ``0`` then go back and check you completed `the last section of the realtime kernel setup`_.

.. tip:: Remember you can check if you are running your realtime kernel at any time by typing ``uname -r`` and looking for an ``rt`` after the kernal version.

.. _`the last section of the realtime kernel setup`: https://frankaemika.github.io/docs/installation.html#allow-a-user-to-set-real-time-permissions-for-its-processes

Test with examples
==================

To start testing you should move to :doc:`operating` page to test out the workstation set up.

Remember, for this to work, you need:

* The FRANKA Arm must be in movement mode (white light).
* The workstation PC must be connected to the shop floor controller by ethernet.

.. tip::
  You can confirm that the workstation computer is able to communicate with the workshop controller by pinging the IP address from the terminal: ``ping 192.168.0.88``. For more information see the :doc:`franka` page.

Install other libraries
=======================

You may want to install OMPL:

http://ompl.kavrakilab.org/installation.html

.. warning::
  The installation of OMPL takes several hours.
