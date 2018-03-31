# import the pygame module, so you can use it
import pygame

class Window(object):
    def __init__(self, caption="", icon=None, size=((200,200))):
        """
        Creates default window that can be inherited and made changes through

        Params
        ------
        caption : string
            String for the title of the window
        icon : string or pygame.surface 
            image to show in title bar. Can be name of image file or image object
            is completely optional and window may have no icon
        size : tuple (size 2)
            gives width x height for the window
        """
        super(Window, self).__init__();
        pygame.init();
        
        if(icon is not None):
            if(type(icon) == str):
                logo = pygame.image.load(icon);
            elif(type(icon) == pygame.surface):
                logo = icon;
            else:
                raise Exception("Icon Type not reconized");
            
            pygame.display.set_icon(logo);

        pygame.display.set_caption(caption);
        self.screen = pygame.display.set_mode(size);
        self.screen.fill((0,0,0));
        self.running = True;
        self.events = {};

    def resetScreen(self):
        self.getScreen().fill((0,0,0))
    
    def getScreen(self):
        return self.screen;
    def getRunning(self):
        return self.running;
    def stopRunning(self):
        self.running = False;

    def addEvent(self, event, response):
        """
        Adds a new event to handle by taking the event object to litsen for
        and using the response function when event is triggered.

        Params
        ------
        event : event type object
            event that is serached for
        reponse : function
            some function that is run in response to the event      
        """
        self.events[event] = response;


    def getEvents(self):
        return self.events;
    def runEvent(self, event):
        self.getEvents()[event]();

    def initalize(self):
        """
        Code for any additional code to apply before the while loop running
        phase.
        Called Seperately
        """
        def closeWindow():
            self.stopRunning();
        self.addEvent(pygame.QUIT, closeWindow );
        
    def preProcessing(self):
        for event in pygame.event.get():
            if event.type in self.getEvents():
                self.runEvent(event.type);
    def postProcess(self):
        pygame.display.flip();
        self.resetScreen();

    def runScript(self):
        """
        Code for what happens during running
        """
        pass;
                

    def run(self):
        """
        Code to run. Starts when you wish
        """
        while(self.getRunning()):
            self.preProcessing();
            self.runScript();
            self.postProcess();
            
            
            
class test1(Window):
    def __init__(self, caption="", icon=None, size=((200,200))):
        super(test1, self).__init__(caption, icon, size);
        

    def initalize(self):
        super(test1, self).initalize();

    def runScript(self):
        pygame.draw.rect(self.getScreen(), (255,0,0), pygame.Rect(0,0,150,70));
        pygame.draw.rect(self.getScreen(), (0,255,0), pygame.Rect(50,50,100,100));
        pygame.display.flip();
        


    
if __name__=="__main__":
    w = test1();
    w.initalize();
    w.run();
