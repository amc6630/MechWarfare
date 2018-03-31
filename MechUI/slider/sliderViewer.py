import pygame
import sliderViewerTactic

class SliderViewer(object):
    def __init__(self, slider, sliderProp, rectangularView):
        super(SliderViewer, self).__init__();
        self.slider = slider;
        self.sliderProp = sliderProp;
        self.rectangularView = rectangularView;
        self.sliderProp.initialize(self.rectangularView);
        self.sliderViewerTactic = sliderViewerTactic.SliderViewerTactic(self.rectangularView, self.slider, self.sliderProp);
        # setup rectangularView to match slider prop data

        
    def getSliderViewerTactic(self):
        return self.sliderViewerTactic;
    def setSliderViewerTactic(self, tactic):
        # shold be never used since tactic has no state and only immutable functions
        self.sliderViewerTactic = tactic;

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
                           self.getSliderViewerTactic().getCursorPosition(),
                           self.getSliderViewerTactic().getRadius());
        

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
        x,y,w,h = self.getRectangularView().extractProperties();
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
