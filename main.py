from __future__ import print_function
import cv2
import time
import camera_subscriber
import argparse
from perception.mainDetect import Perception
from chess.engine import ChessEngine

def main(static):
    """
    Main program for testing
    """

    '''
    0. Start up chess engine
    '''

    engine = ChessEngine(debug=True, suppress_sunfish=False)

    '''
    1. Start by getting picture of empty chessboard
    '''

    # Start camera feed
    feed = camera_subscriber.CameraFeed()
    feed.start_process()
    print("Camera Feed started")

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

    '''
    2. Instantiate Perception object

    Generates Board class within the Perception object holding information about the 64 squares
    '''

    # Make Perception instance
    percept = Perception()

    # Make a Board instance within Perception. This assigns the grid and the initial BWE given an image of an empty board

    #try:
    percept.makeBoard(empty, depthEmpty)
    #except Exception as e:
    #    print("Board could not be instantiated. Error message: " + str(e))

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

    # Counter to determine if it's Opponents (False) or Robots (True) turn
    move = False

    while True:

        # Wait until key pressed
        print("When you have made a move, please press any key to update the BWE matrix.")
        print("")
        try:
            cv2.waitKey(0)
        except KeyboardInterrupt:
            cv2.destroyAllWindows()

        # Refresh rate of camera frames
        time.sleep(0.05)

        # Get current image
        current, dummy = feed.get_frames()

        cv2.imshow("Current Image", current)

        # Update BWE
        bwe = percept.bwe(current, debug=True)

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

        if move == False:
            status, msg = engine.input_bwe(bwe_converted)
            moveCounter += 1

            print("")
            print("Chess Engine Return:")
            print("The status is: ", status)
            print("The message is: ", msg)
            print("")
        else:
            moveCounter -= 1






        cv2.waitKey(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Perception Testing")
    parser.add_argument('-s', '--static', action='store_true', help='Initialising board from a saved and static image')
    args = parser.parse_args()

    try:
        main(args.static)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()