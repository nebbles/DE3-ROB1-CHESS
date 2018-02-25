**************
Controller API
**************

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

  * Action command object - takes each command and builds a motion plan around it
  * Takes in current robot Arm data and modifies motion plan accordingly

* Outputs:

  * Stores motion plan which gets passed to the publisher node
