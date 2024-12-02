# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 16:30:03 2023

@author: Jason
"""

import math
import numpy as np

class GPS_Data:
    
    def __init__(self, seq, timestamp: int, frame_id: tuple, coordinates: np.array(float), quaternion: np.array(float),
                 velocity: np.array(float), omega: np.array(float), sensor_name = None):
        self.seq = seq
        self.timestamp = timestamp
        self.frame_id = frame_id
        self.coordinates = coordinates
        self.quaternion = quaternion
        self.velocity = velocity
        self.omega = omega
        self.sensor_name = sensor_name
        
    def get_coordinates(self):
        return self.coordinates
    
    def get_quaternion(self):
        return self.quaternion
    
    def get_velocity(self):
        return self.velocity
    
    def get_omega(self):
        return self.omega
    
    def get_timestamp(self):
        return self.timestamp
    
    def get_sensor_name(self):
        if self.sensor_name == None:
            self.sensor_name = "PARKER_3DM-GQ7"
        
        return self.sensor_name
        
    def get_angles(self):
       x = self.quaternion[0]
       y = self.quaternion[1]
       z = self.quaternion[2]
       w = self.quaternion[3]
       
       # Roll (x-axis rotation)
       t0 = +2.0 * (w * x + y * z)
       t1 = +1.0 - 2.0 * (x**2 + y**2)
       roll = math.atan2(t0, t1)
   
       # Pitch (y-axis rotation)
       t2 = +2.0 * (w * y - z * x)
       t2 = +1.0 if t2 > +1.0 else t2
       t2 = -1.0 if t2 < -1.0 else t2
       pitch = math.asin(t2)
   
       # Yaw (z-axis rotation)
       t3 = +2.0 * (w * z + x * y)
       t4 = +1.0 - 2.0 * (y**2 + z**2)
       yaw = math.atan2(t3, t4)

       return np.array([roll, pitch, yaw])
    
    
    #This function transforms gps sensor position to the contact location between the tire and the ground
    def get_coordinate_offsets(self, angles):
        z_offset = 2.05 #meters
        
        # Assumptions:
        # +x is forward, +y is left, +z is up
        # Angles are measured using xyz convention above, relative to the horizon
        # Angles are given in radians
        roll = angles[0]
        pitch = angles[1]
        
        dx = z_offset * math.sin(pitch)
        dy = z_offset * math.sin(roll)
        
        return np.array([dx, dy, -z_offset])
        
        
        
        