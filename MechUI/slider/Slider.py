
import pygame

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




    







        
    


    

