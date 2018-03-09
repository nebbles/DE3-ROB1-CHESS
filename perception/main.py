from __future__ import print_function
import cv2
import time
import camera_subscriber
from perception.mainDetect import Perception

def main():
    """
    Main program for testing
    """

    '''
    1. Start by getting picture of empty chessboard
    '''

    # Start camera feed
    feed = camera_subscriber.CameraFeed()
    feed.start_process()
    print("Camera Feed started")

    # Get picture of empty chessboard
    empty, depthEmpty = feed.get_frames()
    print("Image fetched")

    '''
    2. Instantiate Perception object

    Generates Board class within the Perception object holding information about the 64 squares
    '''

    # Make Perception instance
    percept = Perception()

    # Make a Board instance within Perception. This assigns the grid and the initial BWE given an image of an empty board
    try:
        percept.makeBoard(empty, depthEmpty)
    except Exception as e:
        print("Board could not be instatiated. Error message: " + str(e))

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
    needs to run in a loop so that the BWE is updated forever
    '''

    # Get picture of populated chessboard
    populated, dummy = feed.get_frames()

    # Get frame of populated chessboard
    percept.initialImage(populated)

    print("Initial image of populated board assigned and BWE initialised")
    print("")

    '''
    4. Main loop
    '''

    while True:

        # Wait until key pressed
        print("When you have made a move, please press any key to update the BWE matrix.")
        print("")
        cv2.waitKey(0)

        # Refresh rate of camera frames
        time.sleep(0.05)

        # Get current image
        current, dummy = feed.get_frames()

        cv2.imshow("Current Image", current)

        # Update BWE
        bwe = percept.bwe(current, debug=True)

        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()