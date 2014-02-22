#!/usr/bin/env python
import os
import sys
import time

import pygame
from pygame.locals import *
import roslib
import rospy

import base_control_node
def main():
    pygame.init()
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption('jeeves_keyboard_teleop')
    pygame.mouse.set_visible(0)

    g_quit = False
    try:
        while(not g_quit):
            for event in pygame.event.get():
                if event.type == QUIT:
                   print "Caught Pygame.QUIT...exiting"
                   g_quit = True   # end program.

                elif event.type == KEYDOWN and event.key == K_w:
                    print "caught w"

                elif event.type == KEYDOWN and event.key == K_s:
                    print "caught s"
            time.sleep(0.1)

    except KeyboardInterrupt:
        print "Caught KeyboardInterrupt...exiting."
    pygame.quit()

if __name__ == '__main__':
    main()
