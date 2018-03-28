from __future__ import print_function
import sys
import time
import argparse
import cv2
import camera_subscriber
from perception.mainDetect import Perception
from chess.engine import ChessEngine
from franka.franka_control_ros import FrankaRos
from motion import MotionPlanner
# noinspection PyUnresolvedReferences
import rospy


def bwe_converter(bwe):

    bwe_converted = []
    for i in bwe:
        for j in i:
            if j == 2:
                bwe_converted.append('B')
            elif j == 1:
                bwe_converted.append('W')
            elif j == 0:
                bwe_converted.append('E')
            else:
                raise ValueError

    return bwe_converted


def main(static):
    """
    Main program for testing
    """

    '''
    1. Setup

    Initialise ROS, FRANKA and the camera feed. Clock feed hasn't been added yet.
    '''

    # Create ROS node for this project runtime
    rospy.init_node('chess_project_node', anonymous=True)
    
    # Create an object of the Franka control class
    arm = FrankaRos()

    # Create a planner object for executing chess moves
    planner = MotionPlanner(visual=False, debug=True)

    #  0. Start up chess engine
    engine = ChessEngine(debug=True, suppress_sunfish=False)

    time.sleep(1)

    # Start camera feed
    feed = camera_subscriber.CameraFeed()
    time.sleep(1)
    print("Camera Feed started")
    print("")

    # Get picture of empty chessboard

    empty, depth_empty = feed.get_frames()
    if static:
        empty = cv2.imread('perception/empty.jpg', 1)

    print("Image fetched")
    print("")

    # Start Clock feed
    # clock = ClockFeed()
    # clock.start_process()
    # print("Chess clock initialised")
    # print("")

    '''
    2. Instantiate Perception object

    Generates Board class within the Perception object holding information about the 64 squares
    '''

    # Make Perception instance
    perception = Perception()

    # Make a Board instance within Perception. This assigns the grid and the initial BWE given an
    #  image of an empty board
    perception.makeBoard(empty, depth_empty)

    # Wait for user input
    print("Picture of empty board taken and Board instantiated. Please press any key after you "
          "have populated the board")
    cv2.waitKey(0)
    print("\nContinuing...\n")

    '''
    3. Populate board

    The board now needs to be populated in the normal setup. previous is initialised to the image 
    with the populated chessboard with pieces in the start positions. Current is the picture 
    taken after a move has been made. This runs in a loop so that the BWE is updated forever.
    '''

    # Get picture of populated chessboard
    populated, dummy = feed.get_frames()

    # Assign populated image as first 'previous' image, i.e. to compare with current image
    perception.previous = populated

    print("Initial image of populated board assigned and BWE initialised!\n")
    print("The game has started!\n")

    # Close 'Board Identified' Window
    cv2.destroyAllWindows()

    # Show populated board
    cv2.imshow("Populated Board", populated)

    # MAIN LOOP
    # Clock integration still pending.

    robot_move = False  # flag to track who's turn it is
    while True:
        # Wait until key pressed
        print("When you have made the move, please press any key to update the BWE matrix.\n")
        try:
            cv2.waitKey(0)
        except KeyboardInterrupt:
            cv2.destroyAllWindows()

        # END of OPPONENT turn / A clock stops / B clock starts
        # clock.sig_q.put(1)

        # Refresh rate of camera frames
        time.sleep(0.05)

        # Get current image
        current, dummy = feed.get_frames()
        # Show current image
        cv2.imshow("Current Image", current)

        # Update BWE
        bwe, success = perception.bwe(current, debug=True)
        bwe_converted = bwe_converter(bwe)

        if not robot_move:  # The opponent's turn
            if success:  # If the BWE has been updated
                # Now it's the robots turn
                robot_move = True

                # Get new move from Chess Engine
                status, msg = engine.input_bwe(bwe_converted)

                # Let Chess Engine finish
                time.sleep(0.05)

                # Print chess engine return
                print("\nChess Engine Return:")
                print("The status is: ", status)
                print("The message is: ", msg)
                print("")
                if status < 1:
                    print("An invalid move was sent to Sunfish. Exiting...")
                    sys.exit()
                print("Executing chess motion...")
                # msg = [('n', 'h1g3')]  # example of chess move
                planner.input_chess_move(arm, msg)

                # END of ROBOT turn / B clock stops / A clock starts
                # clock.sig_q.put(2)

            # If success is False, the whole thing just runs again :)
            else:
                print("\nERROR: Move has not been recognised\nPlease try the move again!\n")
        else:
            robot_move = False  # The robots turn
            print("\nIt's time for the user to make their move!\n")

        # Don't know exactly why this is needed
        cv2.waitKey(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Perception Testing")
    parser.add_argument('-s', '--static', action='store_true', help='Initialising board from a '
                                                                    'saved and static image')
    args = parser.parse_args()

    try:
        main(args.static)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
