import numpy as np

class DAQInterpreter(object):
    def __init__(self):
        self.channel_names = {  0: 'clock',
                                1: 'lmr',
                                2: 'freq',
				3: 'lpr',
				4: 'empty',
				5: 'xpos',
				6: 'ypos',
                              }
        
        self.channel_param = { 'clock': {'min': 0, 'max': 5},
                               'lmr': {'min': 0, 'max': 5},
			       'lpr': {'min': 0, 'max': 5},
			       'freq': {'min': 0, 'max': 5},
                               'empty': {'min': 0, 'max': 5},
			       'xpos': {'min': 0, 'max': 5},
			       'ypos': {'min': 0, 'max': 5},
                                }
	self.maxt = 30

    def interpret_channel(self, channel, value):
        name = self.channel_names[channel]
        function = self.__getattribute__(name)
        interpreted_value = function(value)
        return name, interpreted_value
    
    def clock(self, raw_value):
        actual_value = (raw_value/2.)   
        return actual_value 

    def lmr(self, raw_value):
        actual_value = (raw_value/2.)   
        return actual_value 
    
    def lpr(self, raw_value):
        actual_value = (raw_value/2.)   
        return actual_value 

    def freq(self, raw_value):
        actual_value = (raw_value/2.)   
        return actual_value 
    
    def empty(self, raw_value):
        actual_value = (raw_value)   
        return actual_value 
    
    def xpos(self, raw_value):
        actual_value = (raw_value)    
        return actual_value 
    
    def ypos(self, raw_value):
        actual_value = (raw_value)    
        return actual_value 
