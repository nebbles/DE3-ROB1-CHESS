# This source code is derived from by Scrambled Code Studios (16 Aug 2011).
# See clock_acknowledgements.txt for licenses and copyrights of original work.
import pygame
from pygame.locals import *
import sys
import multiprocessing as mp
import time


class ClockFeed:
    def __init__(self, debug=False):
        self.debug = debug
        # Initialise Queues holding rgb and depth image
        self.sig_q = mp.Queue(maxsize=1)
        self.clock_feed = None

    def start_process(self):
        # Create process object
        self.clock_feed = mp.Process(target=main, args=(self.sig_q,))
        # Set the daemon to True - causes process to shutdown if parent process shuts
        self.clock_feed.daemon = True
        # Start the process
        self.clock_feed.start()
        # Sleep to allow process to start
        time.sleep(1)


def load_image(name, color_key=None):
    fullname = name
    try:
        image = pygame.image.load(fullname)
    except pygame.error:
        print('Cannot load image:', name)
        raise SystemExit
    image = image.convert()
    if color_key is not None:
        if color_key is -1:
            color_key = image.get_at((0, 0))
        image.set_colorkey(color_key, RLEACCEL)
    return image, image.get_rect()


def main(sig_q):
    sig = None

    time_a = 0
    time_b = 0
    a_on = False
    b_on = False

    pygame.init()
    pygame.display.init()
    screen = pygame.display.set_mode((1280, 720))
    pygame.display.set_caption("Chess Clock")

    background = pygame.Surface(screen.get_size())
    rect = background.fill((0, 0, 0))

    clock_image, clock_rect = load_image("chess/chess_clock/clock_image_new.png")

    clock = pygame.time.Clock()

    font = pygame.font.Font("chess/chess_clock/clock_font_digi.TTF", 120)

    while True:
        clock.tick(30)

        for event in pygame.event.get():

            # Get signal from ClockFeed
            if sig is None:
                try:
                    sig = sig_q.get(False)
                    print("Got signal: ", sig)
                # If the queue is empty
                except:
                    pass

            try:
                if event.type == QUIT:
                    sys.exit()

                if event.type == USEREVENT:
                    if time_a > 0:
                        time_a -= 1
                    else:
                        pygame.time.set_timer(USEREVENT, 0)

                elif event.type == (USEREVENT + 1):
                    if time_b > 0:
                        time_b -= 1
                    else:
                        pygame.time.set_timer(USEREVENT, 0)

                elif event.type == KEYDOWN or sig == 1 or sig == 2:

                    if event.key == K_a or sig == 1:
                        if not a_on:
                            # Set for 1 second (1000 milliseconds)
                            pygame.time.set_timer(USEREVENT, 1000)
                            a_on = True
                        else:
                            # The other one should turn on immediately
                            pygame.time.set_timer(USEREVENT, 0)
                            pygame.time.set_timer(USEREVENT + 1, 1000)
                            b_on = True
                            a_on = False
                        print("Signal has triggered A")

                    if event.key == K_b or sig == 2:
                        if not b_on:
                            pygame.time.set_timer(USEREVENT + 1, 1000)
                            b_on = True
                        else:
                            pygame.time.set_timer(USEREVENT + 1, 0)
                            pygame.time.set_timer(USEREVENT, 1000)
                            a_on = True
                            b_on = False
                        print("Signal has triggered B")

                    sig = None
                    print("Signal has been reset")

                    if event.key == K_q:
                        time_a += 60  # add a minute from allotted time

                    if event.key == K_z:
                        time_a -= 60  # subtract a minute

                    if event.key == K_g:
                        time_b += 60

                    if event.key == K_n:
                        time_b -= 60

                    if event.key == K_PAUSE or event.key == K_p:
                        # pause both timers
                        pygame.time.set_timer(USEREVENT + 1, 0)
                        pygame.time.set_timer(USEREVENT, 0)
            except:
                pass

        # Format time into minutes:seconds
        time_a_str = "%d:%02d" % (int(time_a / 60), int(time_a % 60))
        time_b_str = "%d:%02d" % (int(time_b / 60), int(time_b % 60))

        time_a_txt = font.render(time_a_str, 1, (255, 255, 255))
        time_b_txt = font.render(time_b_str, 1, (255, 255, 255))

        time_a_rect = time_a_txt.get_rect()
        time_a_rect.center = (360, 383)
        time_b_rect = time_b_txt.get_rect()
        time_b_rect.center = (920, 383)

        screen.blit(background, rect)
        screen.blit(clock_image, clock_rect)
        screen.blit(time_a_txt, time_a_rect)
        screen.blit(time_b_txt, time_b_rect)

        pygame.display.update()
