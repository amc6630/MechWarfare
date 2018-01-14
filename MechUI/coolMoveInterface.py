#! /usr/bin/python

#Jeremy Lim
#python script that creates a nice gui for robot control!
# Tested successfully with the logitech Dual Action controller.

#custom imports
from valuePair import LabelPair
from displayTerminal import SurfaceTerminal
from videoSurface import VideoSurface

#import numpy as np
import struct
import socket

import pygame
from pygame.locals import *

from time import sleep

#some global constants
const_text_color = (255,255,255)

def main():
    pygame.init()

    scr_size = 500, 400

    #setup joysticks
    pygame.joystick.init()
    contr1 = pygame.joystick.Joystick(0)
    contr1.init()

    #print contr1.get_name()
    #print contr1.get_numaxes()

    #Fullscreen seizes the screen, but messes up on virtual
    #machines: pygame.FULLSCREEN
    screen = pygame.display.set_mode((0,0))#scr_size

    #white background
    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill((0, 0, 0))

    #Add the video display
    #"http://192.168.42.1:8080/?action=snapshot"
    #vidFeed = VideoSurface("http://192.168.42.1:8080/?action=snapshot",(200,50));


    #xPair = LabelPair((50,50),"X: ")
    #xPair.setDispNumber(0)

    #yPair = LabelPair((100,50),"Y: ")
    #yPair.setDispNumber(0)

    #xPair.blitLabel(background)
    #yPair.blitLabel(background)

    #Key events repeat
    pygame.key.set_repeat(10,10)
    #make the cursor invisible.
    #pygame.mouse.set_visible(False)

    #Suck up all of the inputs!
    #pygame.event.set_grab(1)

    #set up our socket
    controlSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    #Static parts of the display.

    screen.blit(background, (0, 0))
    pygame.display.flip()

    #Main loop
    while 1:
        #network update
        #Read values from joysticks, send those values.

        yawRot = 0
        pitchRot = 0
        #theX = contr1.get_axis(0)
        #theY = contr1.get_axis(1)

        #range is from -1.0 to 1.0
        #left joystick:      +x
        #-------------------------->
        #|
        #|
        #|
        #|
        #|
        #|   +y
        #\/

        #events update
        for event in pygame.event.get():
            if event.type == QUIT:
                return
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return
        
        rotX = contr1.get_axis(0)
        theX = contr1.get_axis(2)
        theY = contr1.get_axis(3)

        #mouse control experiments
        #delta = pygame.mouse.get_rel()
        #yawRot = -delta[0]*1#arbitrary constant
        #pitchRot = -delta[1]*10#arbitrary constant

        xMove = theX*100.0
        yMove = -theY*100.0
        rotation = -60.0*rotX

        
        #x1, y1 = pygame.mouse.get_pos()
        #sleep(0.02)
        #constrain mouse to a valid position
        #pygame.mouse.set_pos([500,500])
        #x2, y2 = pygame.mouse.get_pos()
        #theX = x1 - 500
        #theY = y1 - 500
        #theX = contr1.get_axis(2)
        #theY = contr1.get_axis(3)
        #xPair.setDispNumber(theX)
        #yPair.setDispNumber(theY)

        toggleValue = 0#np.int8(0)
        #right trigger ("8" button)
        if contr1.get_button(7):
            toggleValue = 0x01

        #build our message
        sendStr = []
        sendStr.append(212)
        sendStr.append(xMove)
        sendStr.append(yMove)
        sendStr.append(rotation)
        sendStr.append(toggleValue)
        #print yawRot
        #print pitchRot
        #might be unneeded.
        #writeStr = np.array(sendStr)
        msgStr = struct.pack('Bbbbb',*sendStr)
        #msgStr = struct.pack('Bbbb',*sendStr)#bytearray(writeStr.tostring())

        #use sendto to send control info.
        #(host, portnum) format for address.
        #sending to turret control port.
        controlSocket.sendto(msgStr, ('192.168.42.1',8090))
        #print len(msgStr)

        background.fill((0, 0, 0))
        #render a value pair.
        #testThing.setDispNumber(funNum)

        #vidFeed.blitVideo(background)
        #termThing.blitTerminal(background)
        #xPair.blitLabel(background)
        #yPair.blitLabel(background)
        screen.blit(background, (0, 0))
        pygame.display.flip()
        #Let's be smart about this...
        #pygame.display.update(vidFeed.getBoundingRect())
        #pygame.display.update(vidFeed.getBoundingRect())
        sleep(1.0/25.0)

    #Quick & dirty way to get labels on the screen.
    def blitString(in_backing, disp_string, in_posX, in_posY):
        theFont = pygame.font.Font(None,20)
        
        surf = disp_font.render(disp_string,0,const_text_color)
        write_rect = surf.get_rect()
        write_rect.top = in_posX
        write_rect.left = in_posY
        in_backing.blit(surf,write_rect)

if __name__ == '__main__': main()
