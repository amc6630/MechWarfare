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

    def getMinPosition(self, rectangularView=None):
        if(rectangularView == None):
            rectangularView = self.getRectangularView();
        boundingBox = rectangularView
        x = boundingBox.getXLoc();
        return x+self.getRadius();
    def getMaxPosition(self, rectangularView=None):
        if(rectangularView ==  None):
            rectangularView = self.getRectangularView();
        boundingBox = rectangularView;
        x,w = boundingBox.getXLoc(), boundingBox.getWidth();
        return x+w-self.getRadius();

    def getCursorRange(self):
        """
        Display range of pixels the cursor center can be placed within.
        """
        return self.getMaxPosition() - self.getMinPosition()

    def positionToPercentage(self,  position, rectangularView=None):
        if(rectangularView == None):
            rectangularView = self.getRectangularView();
        boundingBox = rectangularView;
        x,w = self.getMinPosition(), self.getCursorRange();
        distance = position-x;
        return float(distance)/w;

    def percentageToPosition(self, percentage, rectangularView=None):
        if(rectangularView == None):
            rectangularView = self.getRectangularView();
        boundingBox = rectangularView;
        x,w = self.getMinPosition(), self.getCursorRange();
        distance = percentage*w;
        return distance + x;

    def getCursorPosition(self, rectangularView=None, slider=None):
        if(rectangularView == None):
            rectangularView = self.getRectangularView();
        if(slider == None):
            slider = self.getSlider();
        boundingBox = rectangularView;
        percentage = slider.getSliderProportion();

        y,h = boundingBox.getYLoc(), boundingBox.getHeight();

        midYLoc = y+h/2
        
        return int(self.percentageToPosition(percentage)), int(midYLoc);

    def getRadius(self, rectangularView=None, sliderProp=None):
        if(rectangularView == None):
            rectangularView = self.getRectangularView();
        if(sliderProp == None):
            sliderProp = self.getSliderProp();
        boundingBox = rectangularView;
        h = boundingBox.getHeight();
        diameter = sliderProp.getCursorSizePercentage()*h;
        return int(diameter)/2;
