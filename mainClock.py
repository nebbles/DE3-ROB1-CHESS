from __future__ import print_function
import cv2
import time
import camera_subscriber
import argparse
from perception.mainDetect import Perception
from chess.engine import ChessEngine
from chess.chess_clock.clockTest import ClockFeed

# Start Clock feed
clock = ClockFeed()
clock.start_process()
print("Chess clock initialised")
print("")

while True:
    time.sleep(5)
    clock.sig_q.put(1)

