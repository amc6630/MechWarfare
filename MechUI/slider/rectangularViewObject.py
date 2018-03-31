import pygame
class RectangularViewObject(object):
    """
    object that represents location and size of rectangle in pygame coordinates
    """
    def __init__(self, x, y, w, h):
        """
        Constructor to represent object view location and size

        Params
        ------
        x : top left corner x position
        y : top left corner y position
        w : width of box representing view
        h : height of box representing view
        """
        super(RectangularViewObject, self).__init__();
        isParametersNegative = [i<0 for i in (x,y,w,h)];
        if True in isParametersNegative:
            raise Exception("No parameter value can go below 0");
        self.x = x;
        self.y = y;
        self.w = w;
        self.h = h;
        self.rect = pygame.Rect(x,y,w,h);

    def setRect(self, rect):
        self.rect = pygame.rect;
    def getRect(self):
        return self.rect;

    # replace with observer design patter latter
    def updateRect(self):
        self.rect = pygame.Rect(self.getXLoc(),
                                self.getYLoc(),
                                self.getWidth(),
                                self.getHeight());

    def setXLoc(self, xLoc):
        self.x = xLoc;
        self.updateRect();
    def getXLoc(self):
        return self.x;

    def setYLoc(self, yLoc):
        self.y = yLoc;
        self.updateRect();
    def getYLoc(self):
        return self.y;

    def setWidth(self, width):
        self.w = width;
        self.updateRect();
    def getWidth(self):
        return self.w;

    def setHeight(self, height):
        self.h = height;
        self.updateRect();
    def getHeight(self):
        return self.h;
    
    def extractProperties(self, stringOrder=None):
        x = self.getXLoc();
        y = self.getYLoc();
        w = self.getWidth();
        h = self.getHeight();
        if stringOrder is None:
            objectSequence = [x,y,w,h];
        else:
            objectSequence = [];
            for x in stringOrder:
                if(x == "x"):
                    objectSequence.append(x);
                elif(x == "y"):
                    objectSequence.append(y);
                elif(x == "w"):
                    objectSequence.append(w);
                elif(x == "h"):
                    objectSequence.append(h);
                else:
                    objectSequence.append(None);

        return objectSequence;



    
