# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 22:22:29 2023

@author: Jason
"""

import event

class Mode_Change_Event(event.Event):
    
    def __init__(self, event_topic, event_type, event_timestamp, mode: int):
        super().__init__(event_topic, event_type, event_timestamp)
        self.mode = mode
        
    def get_mode(self):
        return self.mode