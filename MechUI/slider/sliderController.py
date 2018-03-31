import pygame

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
            percentage = self.getViewer().getSliderViewerTactic().positionToPercentage(locX);
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
