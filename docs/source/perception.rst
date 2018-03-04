**********
Perception
**********


Description
===========

The perception module enables recognising moves made using machine vision. It is based on OpenCV3. It takes pictures
as an input using a webcam and then processes these. The output of the perception module, which is subsequently
fed into the chess engine, is a Black White Empty (BWE) matrix. This matrix is returned as an 8 by 8 numpy array, filled
with a zero for empty chess squares, a one if the square is occupied by a white piece, and a two if it's occupied by
a black piece. Because the initial setup of the game is constant, this matrix is sufficient to determine the state
of the game at any time.

Design
======

The perception module is written by us and can provide all the chess functionality we need.

There are several classes, such as Line, Square, Board and Perception. The code works in the following sequence:

1. A picture of an empty board is taken and its grid is determined. 64 Square instances are generated, each holding
information about the position of the square, its current state (at this stage they are all empty), and the centres of
the squares. The 64 squares are held in a Board class, holding all the information about the current state of the game.

2. The chessboard is populated by the user in the usual chess setup. The initial BWE matrix is assigned.

3. When the user has made his or her move, the chess clock is pressed. This triggers a new picture to be taken and compared
to the previous one. The squares that have changed (i.e. a piece has been moved from or to) is analysed in terms of its
RGB color spectrum and assigned a new state.

Limitations
===========

This perception module has limitations, which are mostly in terms of robustness and setup. With more coding effort it should
be able to recognise the chessboard even if it is populated.

Implementation
==============


**Documentation**:

.. automodule:: perception.lineClass
  :members:
  :undoc-members:

.. automodule:: perception.squareClass
  :members:
  :undoc-members:

.. automodule:: perception.mainDetect.Perception
  :members:
  :undoc-members:

.. automodule:: perception.boardClass
  :members:
  :undoc-members:

.. automodule:: perception.mainTest
  :members:
  :undoc-members:

