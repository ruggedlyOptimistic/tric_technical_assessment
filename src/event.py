# -*- coding: utf-8 -*-
"""
Created on Thu Nov 16 21:51:19 2023

@author: Jason
"""

from abc import ABC

class Event(ABC):
    
    def __init__(self, event_topic, event_type, event_timestamp):
        
        self.event_topic = event_topic
        self.event_type = event_type
        #self.event_message = event_message
        self.event_timestamp = event_timestamp
    
    def get_topic(self):
        return self.event_topic
    
    def get_type(self):
        return self.event_type

    def get_timestamp(self):
        return self.event_timestamp
    
class Sensor_Event(Event):
    
    def __init__(self, event_topic, event_type, event_timestamp):
        super().__init__(event_topic, event_type, event_timestamp)
        

    def get_sensor_name(self):
        return self.event_topic

    # @abstractmethod
    # def is_healthy(self):
    #     pass