#! /usr/bin/python

#This defines a class that renders video frames onto a surface

import pygame
import urllib
import cStringIO

class VideoSurface:

    def __init__(self, in_url, in_pos, in_width=640, in_height=480):
        self.display_position = in_pos
        self.image_source = in_url
        self.display_width = in_width
        self.display_height=in_height

    def blitVideo(self, in_backing):
        #Goal: grab video from the mjpg streamer
        #Then, transform, display the image.
        im_file = cStringIO.StringIO(urllib.urlopen(self.image_source).read())
        image = pygame.image.load(im_file)

        image = pygame.transform.scale(image,(self.display_width,self.display_height))

        in_backing.blit(image,self.display_position)

    def getBoundingRect(self):
        return pygame.Rect(self.display_position,(display_width,display_height))

