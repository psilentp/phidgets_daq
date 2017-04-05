import numpy as np

class DAQInterpreter(object):
    def __init__(self):
        self.channel_names = {  0: 'clock',
                                1: 'lmr',
                                2: 'freq',
				                3: 'lpr',
				                4: 'chrimson',
				                5: 'xpos',
				                6: 'ypos',
                              }
        
        self.channel_param = { 'clock':    {'min': 0, 'max': 5},
                               'lmr':      {'min': 0, 'max': 5},
			                   'lpr':      {'min': 0, 'max': 5},
			                   'freq':     {'min': 0, 'max': 5},
                               'chrimson': {'min': -10, 'max': 10},
			                   'xpos':     {'min': -10, 'max': 10},
			                   'ypos':     {'min': -10, 'max': 10},
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
    
    def chrimson(self, raw_value):
        actual_value = ((raw_value-499)*(10.0/136))  
        return actual_value 
    
    def xpos(self, raw_value):
        actual_value = ((raw_value-499)*(10.0/136))
        return actual_value 
    
    def ypos(self, raw_value):
        actual_value = ((raw_value-499)*(10.0/136))
        return actual_value 