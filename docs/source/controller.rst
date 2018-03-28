**********
Controller
**********

The controller interfaces the different modules and in this way ensures the flow of the chess game. It handles the whole process from setting up the game and calibrating FRANKA through to to playing the actual game. The board is empty when main.py is started. The user is subsequently required to populate the board and the game is started. After having made a move, the user presses a keyboard button to trigger the chess clock. The move is detected and passed to Sunfish, which replies with the optimal next move. A motion plan for the move is generated and executed by the robot.

.. Note::
 Automatic calibration and the chess clock integration were not finished

Main.py File
============

.. figure:: _static/system_diagram_2.png
    :align: center
    :figclass: align-center

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
