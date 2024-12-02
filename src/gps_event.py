# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 12:54:49 2023

@author: Jason
"""

import event
import gps_data

class GPS_Event(event.Event):
    
    def __init__(self, event_topic, event_type, event_timestamp, data: gps_data):
        super().__init__(event_topic, event_type, event_timestamp)
        self.data = data
        self.topic_timestamp = event_timestamp
        self.event_timestamp = data.get_timestamp()
    
    # Must be a gps_data object
    # By simply passing in this object, we can use its methods
    def get_data(self):
        return self.data
    
    def get_topic_publish_timestamp(self):
        return self.topic_publish_timestamp