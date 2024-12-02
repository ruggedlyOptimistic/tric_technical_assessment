# -*- coding: utf-8 -*-
"""
Created on Mon Nov 20 02:43:40 2023

@author: Jason
"""

from abc import abstractmethod
import event

class Sensor_Event(event.Sensor_Event):
    
    def __init__(self, event_topic, event_type, event_timestamp, sensor_name):
        super().__init__(event_topic, event_type, event_timestamp)
        self.sensor_name = sensor_name

    def get_sensor_name(self):
        return self.sensor_name

    def get_data(self):
        """
        Abstract method to be implemented by subclasses.
        This method should return the specific data associated with the sensor event.
        """
        
