**********
Controller
**********

Using Test Scripts
==================

Throughout our project we used test scripts. These can be seen in the ``tests`` folder. To run these tests properly (e.g. ``test_camera.py``) you should type the following into the terminal::

  cd DE3-ROB1-CHESS/
  python -m tests.test_camera.py

This is to ensure relative imports work properly, as every import is relative to the project level directory.

Main File
=========


.. todo:: redo and add system diagram

Below is the specification for the modules of the Chess Project Python Program. This is to ensure that each module fulfills its function and that the overall goal of the project is achieved. Each module can then seperately be improved and updated, assuming compliance with this specification.

**Clock** module:

* Inputs:

  * Initiate module
  * Button press (type: mouse press)

* Outputs:

  * Up to date time for each player sent to display

**Perception** module:

* Inputs:

  * Initiate module
  * Camera feed (internal to module)
  * Global coordinates
  * Game Engine > Action command object:

    * Converts and saves all locations to real-world coordinates

* Outputs:

  * BWE Matrix (type: numpy array)
  * Synchronisation position of End effector?

**Game Engine** module:

* Inputs:

  * BWE Matrix (type: numpy array)

* Outputs:

  * Action command object:

    * Stores whether or not there is a death first
    * Stores the piece type of death
    * Stores location of death
    * Stores move piece type
    * Stores action-move (piece type)
    * Stores action-move-from (location)
    * Stores action-move-to (location)

**Motion** module:

* Inputs:

  *  Inner corner xyz locations from the perception module
  *  Action command object:
    * Whether or not there is a death first
    * The piece type of death
    * Location of death
    * Move piece type
    * Action-move (piece type)
    * Action-move-from (location)
    * Action-move-to (location)

* Outputs:

  * Trajectory and gripping commands
