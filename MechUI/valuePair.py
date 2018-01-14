#! /usr/bin/python

#This will define a class that implements 
# a text label with a number next to it
# in pygame.

import pygame

class LabelPair:

    def __init__(self, in_pos, in_string, in_color_default = (255, 255, 255), in_size = 20, in_good_color=None, in_good_thresh=0, in_bad_color=None, in_bad_thresh=0):
        #initialize members
        self.display_position = in_pos
        self.display_string = in_string
        self.display_num = 0.0
        self.font_size = in_size

        self.default_color = in_color_default
        if (in_good_color is not None):
            self.good_color = in_good_color
            self.good_thresh = in_good_thresh
        else:
            self.good_color = None
        if (in_bad_color is not None):
            self.bad_color = in_bad_color
            self.bad_thresh = in_bad_thresh
        else:
            self.bad_color = None

        self.disp_font = pygame.font.Font(None,self.font_size)

    def blitLabel(self, in_backing ):
        #blit our text onto the background
        the_color = self.default_color
        if (self.good_color is not None):
            if(self.display_num > self.good_thresh):
                the_color = self.good_color

        if (self.bad_color is not None):
            if(self.display_num < self.bad_thresh):
                the_color = self.bad_color

        surf = self.disp_font.render((self.display_string+str(round(self.display_num,3))),0,the_color)
        write_rect = surf.get_rect()
        write_rect.top = self.display_position[0]
        write_rect.left = self.display_position[1]
        in_backing.blit(surf,write_rect)
        

    def setPos (self, in_pos ):
        self.display_position = in_pos

    def setDispString(self, in_string ):
        self.display_string = in_string

    def setDispNumber(self, new_num ):
        self.display_num = new_num

    def getBoundingRect(self):
        #return pygame.Rect(self.display_position, (self.disp_font.size(self.display_string+str(round(self.display_num,3))),self.disp_font.get_linesize()))
        #num = self.disp_font.size(self.display_string+str(round(self.display_num,3)))
        #print num
        return pygame.Rect(self.display_position, self.disp_font.size(self.display_string+str(round(self.display_num,3))))

