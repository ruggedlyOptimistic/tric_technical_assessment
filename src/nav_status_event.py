# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 23:04:16 2023

@author: Jason
"""
    
import event
from datetime import datetime, timedelta

class Nav_Status_Event(event.Sensor_Event):
   
   def __init__(self, event_topic, event_type, event_timestamp, filter_state, dynamics_mode, status_flags):
       super().__init__(event_topic, event_type, event_timestamp)
       self.filter_state = filter_state
       self.dynamics_mode = dynamics_mode
       self.status_flags = status_flags
       
       self.is_running = False
       self.last_event_time = None
       self.total_running_time = timedelta()
       self.total_unhealthy_time = timedelta()
   
   def get_filter_state(self):
       return self.filter_state
   
   def get_dynamics_mode(self):
       return self.dynamics_mode
   
   def get_status_flags(self):
       return self.status_flags
   
   