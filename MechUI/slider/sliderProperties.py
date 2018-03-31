import pygame
import window

class SliderProperties(object):
    def __init__(self):
        super(SliderProperties, self).__init__();
        self.barWidth = 50;
        self.barHeight = 10;
        self.cursorSizePercentage = 1.50;
        self.barColor = (0,255,0);
        self.cursorColor = (255,255,255);

        self.textVerticleSpacing = 0#10;
        self.textHorizontalRelativeOffset = 0#10;
        self.textWidth = 0#40;
        self.textHeight = 0#20;
        
        self.xOffset = 0#10;
        self.yOffset = 0#10;

        

    def initialize(self, rectangularView):
        
        fullCaptionWidth = self.getTextHorizontalRelativeOffset()+self.getTextWidth();
        contentWidth = max(self.getBarWidth(), fullCaptionWidth);
        width = self.getXOffset() + contentWidth;

        contentHeight = self.getBarHeight() + self.getTextVerticleSpacing() + self.getTextHeight();
        height = self.getYOffset() + contentHeight;
        
        rectangularView.setWidth(width);
        rectangularView.setHeight(height);

    def setBarWidth(self, barWidth):
        self.barWidth = barWidth;
    def getBarWidth(self):
        return self.barWidth;

    def setBarHeight(self, barHeight):
        self.barHeight = barHeight;
    def getBarHeight(self):
        return self.barHeight;

    def setCursorSizePercentage(self, sizePercentage):
        self.cursorSizePercentage = sizePercentage;
    def getCursorSizePercentage(self):
        return self.cursorSizePercentage;

    def setBarColor(self, barColor):
        self.barColor = barColor;
    def getBarColor(self):
        return self.barColor;

    def setCursorColor(self, cursorColor):
        self.cursorColor = cursorColor;
    def getCursorColor(self):
        return self.cursorColor;

    def setTextVerticleSpacing(self, spacing):
         self.textVerticleSpacing = spacing;
    def getTextVerticleSpacing(self):
        return self.textVerticleSpacing;
    def setTextHorizontalRelativeOffset(self, offset):
        self.textHorizontalRelativeOffset = offset;
    def getTextHorizontalRelativeOffset(self):
        return self.textHorizontalRelativeOffset;
    def setTextWidth(self, width):
        self.textWidth = width;
    def getTextWidth(self):
        return self.textWidth;
    def setTextHeight(self, height):
        self.textHeight = height;
    def getTextHeight(self):
        return self.textHeight;
    def setXOffset(self, offset):
         self.xOffset = offset;
    def getXOffset(self):
        return self.xOffset;
    def setYOffset(self, offset):
        self.yOffset = offset;
    def getYOffset(self):
        return self.yOffset;

    def getX(self, xOrigin):
        return xOrigin + xOffset;
    def getY(self, yOrigin):
        return yOrigin + yOffset;
         

    # implement logical accessors for special pixel to value coorespondancy
