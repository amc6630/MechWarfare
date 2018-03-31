import rectangularViewObject as RVO
import window
import slider
import sliderController
import sliderProperties
import sliderViewer



class test(window.Window):
    def __init__(self, caption="", icon=None, size=((200,200))):
        super(test, self).__init__(caption, icon, size);
        self.sliderView = sliderViewer.SliderViewer(slider.Slider(0,0,100), sliderProperties.SliderProperties(), RVO.RectangularViewObject(50,50,0,0))
        self.sliderController = sliderController.SliderController(self.sliderView)
        self.addEvent(self.sliderController.getEvent1(), self.sliderController.getResponse1());
        self.addEvent(self.sliderController.getEvent2(), self.sliderController.getResponse2());
        self.addEvent(self.sliderController.getEvent3(), self.sliderController.getResponse3());
    def runScript(self):
        self.sliderView.draw(self.getScreen());
        

if __name__ == "__main__":      
    test = test();
    test.initalize();
    test.run();
