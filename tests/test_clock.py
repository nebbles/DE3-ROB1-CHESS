from __future__ import print_function
import time
from chess_clock.clock import ClockFeed

if __name__ == '__main__':
    clock = ClockFeed()  # start clock feed
    clock.start_process()
    print("Chess clock initialised.\n")

    while True:
        time.sleep(5)
        clock.sig_q.put(1)
        time.sleep(5)
        clock.sig_q.put(2)
