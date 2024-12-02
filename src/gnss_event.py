# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 22:58:00 2023

@author: Jason
"""

import event
from datetime import datetime, timedelta

class GNSS_Event(event.Sensor_Event):
    
    def __init__(self, event_topic, event_type, event_timestamp, fix_type, num_sv, sbas_used, dngss_used):
        super().__init__(event_topic, event_type, event_timestamp)
        self.fix_type = fix_type
        self.num_sv = num_sv
        self.sbas_used = sbas_used
        self.dngss_used = dngss_used
        
        self.is_running = False
        self.last_event_time = None
        self.total_running_time = timedelta()
        self.total_unhealthy_time = timedelta()
    
    def get_fix_type(self):
        return self.fix_type
    
    def get_num_sv(self):
        return self.num_sv

    def get_sbas_used(self):
        return self.sbas_used

    def get_dngss_used(self):
        return self.dngss_used
    
    def is_healthy(self):
        return self.fix_type == 6
    
    def update_status(self, fix_type, event_time):
        if self.is_running:
            if fix_type == 6:
                self.total_running_time += event_time - self.last_event_time
            else:
                self.is_running = False
                self.total_unhealthy_time += event_time - self.last_event_time
        else:
            if fix_type == 6:
                self.is_running = True
            # If not healthy, do nothing as the GNSS is already not running

        self.last_event_time = event_time
        
    def get_health_summary(self):
        return [self.total_running_time, self.total_unhealthy_time]
        
    