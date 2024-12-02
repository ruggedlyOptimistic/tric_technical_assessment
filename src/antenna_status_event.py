# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 23:07:35 2023

@author: Jason
"""

import event

class Antenna_Status_Event(event.Event):
    
    def __init__(self, event_topic, event_type, event_timestamp, fix_type, gps_tow, heading, heading_uncertainty, rcv_1_valid, rcv_2_valid, antenna_offsets_valid):
        super().__init__(event_topic, event_type, event_timestamp)
        self.fix_type = fix_type
        self.gps_tow = gps_tow
        self.heading = heading
        self.heading_uncertainty = heading_uncertainty
        self.rcv_1_valid = rcv_1_valid
        self.rcv_2_valid = rcv_2_valid
        self.antenna_offsets_valid = antenna_offsets_valid

    def get_fix_type(self):
        return self.fix_type

    def get_gps_tow(self):
        return self.gps_tow

    def get_heading(self):
        return self.heading

    def get_heading_uncertainty(self):
        return self.heading_uncertainty

    def get_rcv_1_valid(self):
        return self.rcv_1_valid

    def get_rcv_2_valid(self):
        return self.rcv_2_valid

    def get_antenna_offsets_valid(self):
        return self.antenna_offsets_valid
    
    def is_healthy(self):
        return self.fix_type == 2 and self.antenna_offsets_valid == 4
