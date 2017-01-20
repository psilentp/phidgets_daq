import numpy as np

class DAQInterpreter(object):
    def __init__(self):
        self.channel_names = {  0: 'wing_beat_analyzer',
                                1: 'frequency',
                                2: 'odor_pulse',
                              }
        
        self.channel_param = { 'wing_beat_analyzer': {'min': 0, 'max': 5},
                               'frequency': {'min': 0, 'max': 12},
                               'odor_pulse': {'min': 0, 'max': 1},
                                }
	self.maxt = 30

    def interpret_channel(self, channel, value):
        name = self.channel_names[channel]
        function = self.__getattribute__(name)
        interpreted_value = function(value)
        return name, interpreted_value
    
    def wing_beat_analyzer(self, raw_value):
        actual_value = (raw_value/1000.)   
        return actual_value 
    
    def frequency(self, raw_value):
        actual_value = (raw_value/1000.)-0.5    
        return actual_value 
    
    def odor_pulse(self, raw_value):
        actual_value = (raw_value/1000.)-0.5    
        return actual_value 
