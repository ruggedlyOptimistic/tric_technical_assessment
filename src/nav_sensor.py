# -*- coding: utf-8 -*-
"""
Created on Mon Nov 20 03:58:06 2023

@author: Jason
"""

import sensor

class Nav_Sensor(sensor.Sensor):

    def __init__(self, name, )    

    def is_healthy(self):
        return self.filter_state == 4
    
    def update_status(self, filter_state, event_time):
        if self.is_running:
            if filter_state == 4:
                self.total_running_time += event_time - self.last_event_time
            else:
                self.is_running = False
                self.total_unhealthy_time += event_time - self.last_event_time
        else:
            if filter_state == 4:
                self.is_running = True
            # If not healthy, do nothing as the sensor is already not running
    
        self.last_event_time = event_time
    
    def get_health_summary(self):
        return [self.total_running_time, self.total_unhealthy_time]   