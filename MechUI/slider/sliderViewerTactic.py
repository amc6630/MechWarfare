import pygame

class SliderViewerTactic(object):
    def __init__(self, rectangularView=None, slider=None, sliderProp=None):
        super(SliderViewerTactic, self).__init__();
        self.rectangularView = rectangularView;
        self.slider = slider;
        self.sliderProp = sliderProp;

    def getRectangularView(self):
        return self.rectangularView;
    def setRectangularView(self, view):
        self.rectangularView = view;

    def getSlider(self):
        return self.slider;
    def setSlider(self, slider):
        self.slider = slider;

    def getSliderProp(self):
        return self.sliderProp;
    def setSliderProp(self, prop):
        self.sliderProp = prop;

    def getMinPosition(self, rectangularView=None, sliderProp=None):
        if(rectangularView == None):
            rectangularView = self.getRectangularView();
        if(sliderProp == None):
            sliderProp = self.getSliderProp();
        boundingBox = rectangularView
        xPos = boundingBox.getXLoc();
        x = sliderProp.getX(xPos, "bar")
        return x+self.getRadius();
    def getMaxPosition(self, rectangularView=None, sliderProp=None):
        if(rectangularView == None):
            rectangularView = self.getRectangularView();
        if(sliderProp == None):
            sliderProp = self.getSliderProp();
        boundingBox = rectangularView
        xPos = boundingBox.getXLoc();
        x = sliderProp.getX(xPos, "bar")
        w = sliderProp.getBarWidth();
        return x+w-self.getRadius();

    def getCursorRange(self):
        """
        Display range of pixels the cursor center can be placed within.
        """
        return self.getMaxPosition() - self.getMinPosition()

    def positionToPercentage(self,  position):
        x,w = self.getMinPosition(), self.getCursorRange();
        distance = position-x;
        return float(distance)/w;

    def percentageToPosition(self, percentage):
        x,w = self.getMinPosition(), self.getCursorRange();
        distance = percentage*w;
        return distance + x;

    def getCursorPosition(self, rectangularView=None, slider=None, sliderProp=None):
        if(rectangularView == None):
            rectangularView = self.getRectangularView();
        if(slider == None):
            slider = self.getSlider();
        if(sliderProp == None):
            sliderProp = self.getSliderProp();
        boundingBox = rectangularView;
        yPos = rectangularView.getYLoc();
        percentage = slider.getSliderProportion();

        y,h = sliderProp.getX(yPos, "bar"), sliderProp.getBarHeight();

        midYLoc = y+h/2
        
        return int(self.percentageToPosition(percentage)), int(midYLoc);

    def getRadius(self, sliderProp=None):
        if(sliderProp == None):
            sliderProp = self.getSliderProp();
        h = sliderProp.getBarHeight();
        diameter = sliderProp.getCursorSizePercentage()*h;
        return int(diameter)/2;

    def getBarRect(self, rectangularView=None, sliderProp=None):
        if(rectangularView == None):
            rectangularView = self.getRectangularView();
        if(sliderProp == None):
            sliderProp = self.getSliderProp();
        x = sliderProp.getX(rectangularView.getXLoc(),"bar");
        y = sliderProp.getY(rectangularView.getYLoc(),"bar");
        w = sliderProp.getBarWidth();
        h = sliderProp.getBarHeight();

        return pygame.Rect(x,y,w,h);

    def getTextRect(self, rectangularView=None, sliderProp=None):
        if(rectangularView == None):
            rectangularView = self.getRectangularView();
        if(sliderProp == None):
            sliderProp = self.getSliderProp();
        x = sliderProp.getX(rectangularView.getXLoc(),"text");
        y = sliderProp.getY(rectangularView.getYLoc(),"text");
        w = sliderProp.getTextWidth();
        h = sliderProp.getTextHeight();

        return pygame.Rect(x,y,w,h)
