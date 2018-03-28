******
Camera
******

Description
===========

In this project, an RGB-D camera was used to detect FRANKA and the chessboard. This part of the project allows us to fetch a single image from the camera livestream whenever it is needed. It is utilises OpenCV and is interfaced with ROS. RGB and depth information collected from the image frame is used in both automatic calibration procedures and within the Perception module for board and game state recognition.

Design
======
The external RGB-D camera is connected through USB. In order to use OpenNI-compliant devices in ROS and launch the camera drive we need to launch the rospackage for the device:

.. code-block:: shell

  roslaunch openni2_launch openni2.launch

.. warning:: **You must run this on the client machine**. If you run the camera rospackage on the host computer running the FRANKA control loop it will cause the controller to crash.

The camera subscriber was initially written to use Python processes. This gave us a significant boost in speed in our main runtime loop however we updated our version to use a time synchronised thread in order to simplify the code. Functionally, the two versions provide the same output. The full version can be viewed below at the header `Camera Subscriber with Processes`_.

In the version without processes, a class was created which initialised the subscribers as members:

.. literalinclude:: ../../camera_subscriber.py
   :lines: 34-40

The callback function of the class was then used in the synchronised thread to update the members of the class storing the latest rgb and depth image:

.. literalinclude:: ../../camera_subscriber.py
   :lines: 49-53

This meant that during the runtime, when the latest image was needed, it could be fetched from the class (after converting into a CV image):

.. literalinclude:: ../../camera_subscriber.py
  :lines: 55-73

You can download this simpler version of the camera subscriber :download:`here <../../camera_subscriber.py>`.

Camera Subscriber with Processes
================================

:download:`Download the processes version <../../archive/camera_subscriber_processes.py>`

This version consists two classes and process function. The ``CameraFeed`` class creates a seperate process using the ``main()`` function. This function starts up the ``ImageConverter`` class which handles the Subscribers and image conversion into OpenCV format. A queue is created between the main function running in the new process and the original ``CameraFeed`` class running in the usual process.

Multiprocessing was used to spawn multiple subprocesses for parallel execution of tasks. First Queues are initialised holding RGB and depth images that need to be processed, and set the maxsize to ``1``. Therefore, only one image can be held at a time. ``get_frames()`` method is used to to retrieve the results from ``queue`` and return to the caller:

.. literalinclude:: ../../archive/camera_subscriber_processes.py
  :lines: 40-48

In the background process, the callback function is being called. This is attempting to add the latest image to the queue for the parent process to collect it. When it fetches a new image from the subscriber it will place it in the queue. If the queue is alread full, it overwrites the image. This ensures there is only ever one image in the queue. This occurs for both the RGB image queue and the depth image queue.

.. literalinclude:: ../../archive/camera_subscriber_processes.py
  :lines: 68-109
