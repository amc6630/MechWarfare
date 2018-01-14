#! /usr/bin/python

#This will define a class that implements 
# a surface that acts like a terminal, with a maximum number of lines present.
# in pygame.

import pygame

class SurfaceTerminal:

    def __init__(self, in_pos, in_color=(255,255,255),in_size=20,in_font_size=20):
        #initialize members
        self.display_position = in_pos
        self.display_list = []
        self.display_size = 0
        self.max_line_width = 0
        self.display_max_size = in_size
        self.default_color = in_color
        self.font_size = in_font_size

        self.disp_font = pygame.font.Font(None,self.font_size)

    def blitTerminal(self, in_backing):
        temp_pos = self.display_position
        
        for thing in self.display_list:
            surf = self.disp_font.render(thing,0,self.default_color)
            write_rect = surf.get_rect()
            write_rect.top = temp_pos[0]
            write_rect.left = temp_pos[1]
            in_backing.blit(surf,write_rect)
            
            #update the position
            temp_pos = (temp_pos[0]+self.disp_font.get_linesize(),temp_pos[1])

            

    def setPos(self,in_pos):
        self.display_position = in_pos

    def getBoundingRect(self):
        return pygame.Rect(self.display_position,(self.max_line_width,self.display_size*self.disp_font.get_linesize()))
        
    def clearTerminal(self):
        self.display_list.clear()
        sefl.display_size = 0
        
    def appendLine(self, in_line):
        if(self.display_size >= self.display_max_size):
            self.display_list.pop(0)
            self.display_list.append(in_line)
        else:
            self.display_size += 1
            self.display_list.append(in_line)

        #update the max width.
        if (self.disp_font.size(in_line) > self.max_line_width):
            self.max_line_width = self.disp_font.size(in_line)[0]

    #def setLineLimit(in_max):
        

