# -*- coding: utf-8 -*-
"""
Created on Mon Nov 20 03:54:33 2023

@author: Jason
"""

from abc import ABC

class Sensor(ABC):
    
    def __init__(self, name):
        self.name = name
        self.events = []  
    
    def get_name(self):
        return self.name
    
    def get_events(self):
        return self.events
    
    def append_events(self, event):
        self.events.append(event)