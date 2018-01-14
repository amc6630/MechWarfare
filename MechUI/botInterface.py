#! /usr/bin/python

#Jeremy Lim
#python script that creates a nice gui for robot control!

#custom imports
from valuePair import LabelPair
from displayTerminal import SurfaceTerminal
from videoSurface import VideoSurface

#import numpy as np
import socket

import ctypes as ct
import struct

import pygame
from pygame.locals import *

import time

#some global constants
const_text_color = (255,255,255)

move_percentage = 15# When you press a key, what percentage of max speed will the mech move/turn?
Magic_num = 212# byte value at the start of each message. Consistent between server and this control software.

def main():
    pygame.init()

    scr_size = 500, 400

    #Fullscreen seizes the screen, but messes up on virtual
    #machines: pygame.FULLSCREEN
    screen = pygame.display.set_mode((0,0))#scr_size

    #white background
    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill((0, 0, 0))

    #Black text
    #font = pygame.font.Font(None, 36)
    #txtSurface = font.render("Hello THere", 1, (10, 10, 10))
    #txtPos = txtSurface.get_rect()

    #add to background
    #txtPos.centerx = background.get_rect().centerx
    #background.blit(txtSurface, txtPos)

    #Add value pairs
    #testThing = LabelPair((50,50),"The number: ",in_good_color=(0,255,0),in_good_thresh=75,in_bad_color=(255,0,0),in_bad_thresh=25)
    #testThing.setDispNumber(0)

    #Add the terminal
    #termThing = SurfaceTerminal((50,50))

    #Add the video display
    # NOTE: Disabled for now. Will crash if the streamer isn't on and tries to get image.
    #"http://192.168.42.1:8080/?action=snapshot"
    #vidFeed = VideoSurface("http://192.168.42.1:8080/?action=snapshot",(200,50));

    #Key events repeat
    pygame.key.set_repeat(10,10)

    #set up our socket
    controlSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    #Static parts of the display.
    screen.blit(background, (0, 0))
    pygame.display.flip()

    #Main loop
    funNum = 0.0
    increment = 0.01

    #keymap variable.
    #keeps track of when keys were pressed/released.
    keymap = [False]*7
    #Keys to track: w a s d q e x
    xMove = 0
    yMove = 0
    while 1:
        #network update
        #we update value pairs, terminal & video feed.

        rotation = 0
        #events update
        for event in pygame.event.get():
            if event.type == QUIT:
                return
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return
                if event.key == pygame.K_w:
                    keymap[0] = True
                if event.key == pygame.K_s:
                    keymap[2] = True
                if event.key == pygame.K_d:
                    keymap[3] = True
                if event.key == pygame.K_a:
                    keymap[1] = True
                if event.key == pygame.K_e:
                    keymap[5] = True
                if event.key == pygame.K_q:
                    keymap[4] = True
                if event.key == pygame.K_x:
                    keymap[6] = True
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_w:
                    keymap[0] = False
                if event.key == pygame.K_s:
                    keymap[2] = False
                if event.key == pygame.K_d:
                    keymap[3] = False
                if event.key == pygame.K_a:
                    keymap[1] = False
                if event.key == pygame.K_e:
                    keymap[5] = False
                if event.key == pygame.K_q:
                    keymap[4] = False
                if event.key == pygame.K_x:
                    keymap[6] = False
        yMove = 0
        xMove = 0
        rotation = 0
        #TODO: Refactor.
        da_const = move_percentage
        #decisions based on key state.
        if keymap[0]: #w is down
            yMove = da_const#yMove + 1
        if keymap[2]: #s
            yMove = -da_const#yMove - 1
        if keymap[1]: #a
            xMove = -da_const#xMove - 1
        if keymap[3]: #d
            xMove = da_const#xMove + 1
        if keymap[4]: #q
            rotation = da_const#rotation + 2
        if keymap[5]: #e
            rotation = -da_const#rotation - 2

        toggleValue = 0#np.int8(0)

        if keymap[6]:
            toggleValue = 0x80
        #build our message
        sendStr = []
        sendStr.append(212)#np.int8(212)
        sendStr.append(xMove)
        sendStr.append(yMove)
        sendStr.append(rotation)
        sendStr.append(toggleValue)
        #might be unneeded.
        #writeStr = np.array(sendStr)
        #print writeStr
        #print sendStr

        msgStr = struct.pack('Bbbbb',*sendStr)
        #msgStr = bytearray(sendStr)

        #use sendto to send control info.
        #(host, portnum) format for address.
        controlSocket.sendto(msgStr, ('192.168.42.1',8090)) #this address can be changed.
        #controlSocket.sendto(msgStr, ('127.0.0.1',8090))

        background.fill((0, 0, 0))
        #render a value pair.
        
        #termThing.appendLine("Value: "+str(round(funNum,3)))
        
        # update screen objects.

        #vidFeed.blitVideo(background)
        #termThing.blitTerminal(background)
        #testThing.blitLabel(background)
        screen.blit(background, (0, 0))
        #pygame.display.flip()
        #Let's be smart about this...
        #pygame.display.update(vidFeed.getBoundingRect())
        #pygame.display.update(vidFeed.getBoundingRect())
        time.sleep(1.0/25.0)

    #Quick & dirty way to get labels on the screen.
    def blitString(in_backing, disp_string, in_posX, in_posY):
        theFont = pygame.font.Font(None,20)
        
        surf = disp_font.render(disp_string,0,const_text_color)
        write_rect = surf.get_rect()
        write_rect.top = in_posX
        write_rect.left = in_posY
        in_backing.blit(surf,write_rect)

if __name__ == '__main__': main()
