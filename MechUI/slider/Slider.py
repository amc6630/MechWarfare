import rectangularViewObject as RVO
import pygame
import window

class Slider(object):
    def __init__(self, minValue, initialValue, maxValue): # consider replace RVO as member field instead
        """
        Produces a slider data model with BPO (Basic Python Objects) for the
        value of slider relative to slider size

        Params
        ------
        initialValue : double or int
            starting value of the slider
        maxValue : double or int
            largest value of slider
        minValue : double or int
            smallest valud of slider
        """
        super(Slider, self).__init__();
        if not(minValue <= maxValue):
            raise Exception("Max Value must be larger or equal to min value");
        elif not (minValue <= initialValue <= maxValue ):
            raise Exception("initialValue must be between minValue and MaxValue");

        self.minValue = minValue;
        self.maxValue = maxValue;
        self.value = initialValue;
        

    
        

    def getMinValue(self):
        return self.minValue;
    def getMaxValue(self):
        return self.maxValue;
    def getValue(self):
        return self.value;

    def _setMinValue(self, minValue):
        self.minValue = minValue;
    def _setMaxValue(self, maxValue):
        self.maxValue = maxValue;
    def _setValue(self, value):
        self.value = value;

    def setNewState(self, minValue=None, value=None, maxValue=None):
        """
        Builds Slider with all new values, can set None to keep value the same

        Params
        ------
        initialValue : double or int or None
            starting value of the slider
        maxValue : double or int or None
            largest value of slider
        minValue : double or int or None
            smallest valud of slider
            
        """
        if(minValue is None):
            minValue = self.getMinValue();
        if(value is None):
            value = self.getValue();
        if(maxValue is None):
            maxValue = self.getMaxValue();

        if not(minValue <= maxValue):
            raise Exception("Max Value must be larger or equal to min value");
        elif not (minValue <= value <= maxValue ):
            raise Exception("initialValue must be between minValue and MaxValue");

        self._setMinValue(minValue);
        self._setValue(value);
        self._setMaxValue(maxValue);

    def setMinValue(self, minValue):
        self.setNewState(minValue=minValue);
    def setValue(self, value):
        self.setNewState(value=value);
    def setMaxValue(self, maxValue):
        self.setNewState(maxValue=maxValue);

    """
    Uses fraction from 0.0 - 1.0 to represent slider position
    """
    def getSliderProportion(self):
        range = self.getMaxValue()-self.getMinValue();
        distance = self.getValue()-self.getMinValue();
        return float(distance)/range;

    def setSliderProportion(self, proportion):
        range = self.getMaxValue()-self.getMinValue();
        distance = proportion*range;
        self.setValue(self.getMinValue() + distance);

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

class SliderViewer(object):
    def __init__(self, slider, sliderProp, rectangularView):
        super(SliderViewer, self).__init__();
        self.slider = slider;
        self.sliderProp = sliderProp;
        self.rectangularView = rectangularView;
        self.sliderProp.initialize(self.rectangularView);
        # setup rectangularView to match slider prop data

        
        

    def getRectangularView(self):
        return self.rectangularView;
    def setRectangularView(self, rectangularView):
        self.rectangularView = rectangularView;

    def setSlider(self, slider):
        self.slider = slider;
    def getSlider(self):
        return self.slider;

    def setSliderProp(self, sliderProp):
        self.sliderProp = sliderProp;
    def getSliderProp(self):
        return self.sliderProp;

    def getMinPosition(self):
        boundingBox = self.getRectangularView();
        x = boundingBox.getXLoc()
        return x+self.getRadius();
    def getMaxPosition(self):
        boundingBox = self.getRectangularView();
        x,w = boundingBox.getXLoc(), boundingBox.getWidth();
        return x+w-self.getRadius();

    def getCursorRange(self):
        """
        Display range of pixels the cursor center can be placed within.
        """
        return self.getMaxPosition() - self.getMinPosition()

    def positionToPercentage(self, position):
        boundingBox = self.getRectangularView();
        x,w = self.getMinPosition(), self.getCursorRange();
        distance = position-x;
        return float(distance)/w;

    def percentageToPosition(self, percentage):
        boundingBox = self.getRectangularView();
        x,w = self.getMinPosition(), self.getCursorRange();
        distance = percentage*w;
        return distance + x;

    def getCursorPosition(self):
        boundingBox = self.getRectangularView();
        percentage = self.getSlider().getSliderProportion();

        y,h = boundingBox.getYLoc(), boundingBox.getHeight();

        midYLoc = y+h/2
        
        return int(self.percentageToPosition(percentage)), int(midYLoc);

    def getRadius(self):
        boundingBox = self.getRectangularView();
        h = boundingBox.getHeight();
        diameter = self.getSliderProp().getCursorSizePercentage()*h;
        return int(diameter)/2;
    
    def draw(self, screen):
        rect = self.getRectangularView().getRect();

        """
        self.drawShape("rect", screen, self.getSliderProp().getBarColor(), rect);
        self.drawShape("circle", screen,
                           self.getSliderProp().getCursorColor(),
                           self.getCursorPosition(),
                           self.getRadius());
        """
        
        self.drawShape("rect.png","rect", screen, self.getSliderProp().getBarColor(), rect);
        self.drawShape("circle.png","circle", screen,
                           self.getSliderProp().getCursorColor(),
                           self.getCursorPosition(),
                           self.getRadius());
        

        """
        pygame.draw.rect(screen, self.getSliderProp().getBarColor(), rect);
        pygame.draw.circle(screen,
                           self.getSliderProp().getCursorColor(),
                           self.getCursorPosition(),
                           self.getRadius());
        """
        
        
    def drawShape(self, img, *args):
        if(".jpg" in img or ".png" in img or ".bmp" in img):
            image = pygame.image.load(img);

            screen = args[1];
            shape = args[0];
            
            if(shape == "rect"):
                Rect = args[3];
                self.buildRectImage(image, Rect, screen)
            elif(shape == "circle"):
                centerPos, radius = args[3], args[4];
                self.buildCircleImage(image, centerPos, radius, screen);
        elif(img == "rect"):
            screen, color, rect = args[0], args[1], args[2]
            pygame.draw.rect(screen, color, rect);
        elif(img == "circle"):
            screen, color, pos, radius = args[0], args[1], args[2], args[3]
            pygame.draw.circle(screen, color, pos, radius)

    def buildRectImage(self, image, Rect, screen): 
        x,y,w,h = self.getRectangularView().extractProperties();#Rect.x, Rect.y, Rect.width, Rect.height; # no accessor method
        size = (w,h);
                    
        finalImage = pygame.transform.scale(image, size);
        screen.blit(finalImage, (x,y));

    def buildCircleImage(self, image, centerPos, radius, screen):
        """
        Code to draw circle by loading an image of a circle, can also specify circle properties.
        Decoration for better looking circles. Not must be cropped so perfect circle and edges touching edge of image.
        """
        diameter = 2*radius
                
        size = (diameter, diameter);
        finalImage = pygame.transform.scale(image, size);
                    
        xCenter, yCenter = centerPos[0], centerPos[1];
        x = xCenter - radius;
        y = yCenter - radius;

        screen.blit(finalImage, (x,y));

class SliderController(object):
    def __init__(self, viewer):
        super(SliderController, self).__init__();
        self.viewer = viewer;
        self.isHolding = False;

    def setViewer(self, viewer):
        self.viewer = viewer;
    def getViewer(self):
        return self.viewer;

    def moveCursorToMouse(self):
        pos = pygame.mouse.get_pos();
        if(self.getViewer().getRectangularView().getRect().collidepoint(pos)):
            locX = pos[0]
            percentage = self.getViewer().positionToPercentage(locX);
            percentage = 1.0 if(percentage > 1.0) else percentage
            percentage = 0 if (percentage <0.0) else percentage
            self.getViewer().getSlider().setSliderProportion(percentage);
        
    
    def getEvent1(self):
        return pygame.MOUSEBUTTONDOWN;
    def getResponse1(self):
        def eventResponse():
            self.moveCursorToMouse();
            self.isHolding = True;  
        return eventResponse;

    def getEvent2(self):
        return pygame.MOUSEMOTION;
    def getResponse2(self):
        def eventResponse():
            if(self.isHolding):
                self.moveCursorToMouse();
        return eventResponse

    def getEvent3(self):
        return pygame.MOUSEBUTTONUP;
    def getResponse3(self):
        def eventResponse():
            self.isHolding=False;
        return eventResponse

class test(window.Window):
    def __init__(self, caption="", icon=None, size=((200,200))):
        super(test, self).__init__(caption, icon, size);
        self.sliderView = SliderViewer(Slider(0,0,100), SliderProperties(), RVO.RectangularViewObject(50,50,0,0))
        self.sliderController = SliderController(self.sliderView)
        self.addEvent(self.sliderController.getEvent1(), self.sliderController.getResponse1());
        self.addEvent(self.sliderController.getEvent2(), self.sliderController.getResponse2());
        self.addEvent(self.sliderController.getEvent3(), self.sliderController.getResponse3());
    def runScript(self):
        self.sliderView.draw(self.getScreen());
        

if __name__ == "__main__":      
    test = test();
    test.initalize();
    test.run();

        
    


    

