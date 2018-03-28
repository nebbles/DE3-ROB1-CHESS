from __future__ import print_function
import cv2
import sys
import time
import camera_subscriber
import argparse
from perception.mainDetect import Perception
from chess.engine import ChessEngine
from chess.chess_clock.clockTest import ClockFeed
import rospy
from franka.franka_control_ros import FrankaRos
from motion import MotionPlanner

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

    Initialise ROS, FRANKA and the camera feed. Clock feed still needs to be added
    '''

    # Create ROS node for this project runtime
    rospy.init_node('chess_project_node', anonymous=True)
    
    # Create an object of the Franka control class
    arm = FrankaRos()

    # Create a planner object for executing chess moves
    planner = MotionPlanner(arm, visual=False, manual_calibration=False, debug=True)

    #  0. Start up chess engine
    engine = ChessEngine(debug=True, suppress_sunfish=False)

    time.sleep(1)

    
    #  1. Start by getting picture of empty chessboard
    
    # Start camera feed
    feed = camera_subscriber.CameraFeed()
    time.sleep(1)
    print("Camera Feed started")
    print("")

    # Get picture of empty chessboard

    if not static:
        try:
            empty, depthEmpty = feed.get_frames()
        except:
            print("Image could not be fetched. ")
    else:
        empty, depthEmpty = feed.get_frames()
        empty = cv2.imread('perception/empty.jpg', 1)

    print("Image fetched")
    print("")

    # Start Clock feed
    #clock = ClockFeed()
    #clock.start_process()
    #print("Chess clock initialised")
    #print("")

    '''
    2. Instantiate Perception object

    Generates Board class within the Perception object holding information about the 64 squares
    '''

    # Make Perception instance
    percept = Perception()

    # Make a Board instance within Perception. This assigns the grid and the initial BWE given an image of an empty board
    percept.makeBoard(empty, depthEmpty)

    # Wait for user input
    print("Picture of empty board taken and Board instantiated. Please press any key after you have populated the board")
    cv2.waitKey(0)
    print("")
    print("Continuing...")
    print("")

    '''
    3. Populate board

    The board now needs to be populated in the normal setup. previous is initialised to the image with the populated
    chessboard with pieces in the start positions. Current is the picture taken after a move has been made. This
    runs in a loop so that the BWE is updated forever
    '''

    # Get picture of populated chessboard
    populated, dummy = feed.get_frames()

    # Assign populated image as first 'previous' image, i.e. to compare with current image
    percept.previous = populated

    print("Initial image of populated board assigned and BWE initialised!")
    print("")
    print("The game has started!")
    print("")

    # Close 'Board Identified' Window
    cv2.destroyAllWindows()

    # Show populated board
    cv2.imshow("Populated Board", populated)

    '''
    4. Main loop

    Clock integration still pending.
    '''

    # Boolean to determine if it's Opponents (False) or Robots (True) turn
    move = False

    while True:

        # Wait until key pressed
        print("When you have made the move, please press any key to update the BWE matrix.")
        print("")
        try:
            cv2.waitKey(0)
        except KeyboardInterrupt:
            cv2.destroyAllWindows()

        # END of OPPONENT turn / A clock stops / B clock starts
        #clock.sig_q.put(1)

        # Refresh rate of camera frames
        time.sleep(0.05)

        # Get current image
        current, dummy = feed.get_frames()
        # Show current image
        cv2.imshow("Current Image", current)

        # Update BWE
        bwe, success = percept.bwe(current, debug=True)
        bwe_converted = bwe_converter(bwe)

        if move == False: # The opponent's turn

            if success: # If the BWE has been updated
                try:
                    # Get new move from Chess Engine
                    status, msg = engine.input_bwe(bwe_converted)
                    '''
                    HERE MOTION NEEDS TO PUT IN THE MOVE
                    '''
                    # END of ROBOT turn / B clock stops / A clock starts
                    #clock.sig_q.put(2)
                except:
                    print("ERROR: Chess engine or Motion failed!")

                # Now it's the robots turn
                move = True

                # Let Chess Engine finish
                time.sleep(0.05)

                # Print chess engine return
                print("")
                print("Chess Engine Return:")
                print("The status is: ", status)
                print("The message is: ", msg)
                print("")
                if status < 1:
                    print("There was an invlid move sent to Sunfish")
                    sys.exit()
                print("EXECUTE CHESS MOTION")
                # chess_move = [('n', 'h1g3')]  # example of chess move, do not uncomment
                planner.input_chess_move(arm, msg)
                print("")

            # If success is False, the whole thing just runs again :)
            else:
                print("")
                print("ERROR: Move has not been recognised")
                print("Please try the move again!")
                print("")
        else:
            # The robots turn
            move = False
            print("")
            print("!!!Please make your move!!!")
            print("")

        # Don't know exactly why this is needed
        cv2.waitKey(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Perception Testing")
    parser.add_argument('-s', '--static', action='store_true', help='Initialising board from a saved and static image')
    args = parser.parse_args()

    try:
        main(args.static)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()