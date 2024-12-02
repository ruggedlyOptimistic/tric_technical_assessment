# -*- coding: utf-8 -*-
"""
Created on Sun Nov 19 23:30:36 2023

@author: Jason
"""

"""
Road Profile Class

This class models the vertical motion of a robot over time using a sinusoidal forcing function.

Libraries:
- NumPy:      https://numpy.org/doc/1.18/
- SciPy:      https://docs.scipy.org/doc/scipy/reference/
- Matplotlib: https://matplotlib.org/stable/contents.html
"""

import numpy as np
from scipy.optimize import curve_fit
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
import os

class Road_Profile:
    
    def __init__(self, times, positions, velocities, robot_mass=1.0, debug = False):
        """
        Initialize the RoadProfile class with an mx3 array of times, vertical positions, and vertical velocities.

        Parameters:
        - data (ndarray): An mx3 array where each row represents [time, vertical_position, vertical_velocity].
        - robot_mass (float, optional): Mass of the robot. Default is 1.0.
        """
        self.times = times              # assume already in seconds
        self.positions = positions
        self.velocities = velocities
        self.robot_mass = robot_mass
        self.debug = debug

        # Initial guess for the parameters - arbitrary
        self.initial_guess = [1.0, 0.9, 0.0]

        # Fit the model to the data using curve_fit
        self.fit_params, _ = curve_fit(self.objective_function, self.times, self.positions, p0=self.initial_guess)

        # Extract the fitted parameters
        self.fitted_amplitude, self.fitted_angular_frequency, self.fitted_phase_shift = self.fit_params

    def objective_function(self, t, amplitude, angular_frequency, phase_shift):
        """
        Objective function for curve fitting. It represents the difference between the model and experimental data.

        Parameters:
        - t (array): Time values.
        - amplitude (float): Amplitude of the sinusoidal forcing function.
        - angular_frequency (float): Angular frequency of the sinusoidal forcing function.
        - phase_shift (float): Phase shift of the sinusoidal forcing function.

        Returns:
        - residuals (array): Differences between the integrated model and experimental data.
        """
        # Integrate the forcing function to get displacements
        displacements = np.cumsum(np.cumsum(self.sinusoidal_forcing_function(t, amplitude, angular_frequency, phase_shift)) / self.robot_mass)

        # Calculate residuals (difference between model and data)
        residuals = displacements - self.positions

        # Penalize differences in velocities
        residuals += 0.1 * (np.cumsum(self.sinusoidal_forcing_function(t, amplitude, angular_frequency, phase_shift)) / self.robot_mass -
                            self.velocities)

        return residuals

    def sinusoidal_forcing_function(self, t, amplitude, angular_frequency, phase_shift):
        """
        Generate the sinusoidal forcing function.

        Parameters:
        - t (array): Time values.
        - amplitude (float): Amplitude of the sinusoidal forcing function.
        - angular_frequency (float): Angular frequency of the sinusoidal forcing function.
        - phase_shift (float): Phase shift of the sinusoidal forcing function.

        Returns:
        - values (array): Values of the sinusoidal forcing function at the given time points.
        """
        return amplitude * np.sin(angular_frequency * t + phase_shift)

    def get_model_parameters(self):
        """
        return the model parameters as a list
        """
        print("Fitted parameters:")
        print("Amplitude:", self.fitted_amplitude)
        print("Angular Frequency:", self.fitted_angular_frequency)
        print("Phase Shift:", self.fitted_phase_shift)
        
        return [self.fitted_amplitude, self.fitted_angular_frequency, self.fitted_phase_shift]
        
    
    def plot_results(self, image_dir):
        # Plot road profile
        plt.figure(figsize=(12, 8))
        plt.plot(self.times, self.positions, label='Road Profile')
        plt.title('Road Profile')
        plt.xlabel('Time')
        plt.ylabel('Vertical Displacement (meters)')  # Updated ylabel
        plt.grid(True, linestyle="--", alpha=0.7)
        plt.legend()
        
        # Save the road profile plot
        road_profile_path = os.path.join(image_dir, 'road_profile.png')
        plt.savefig(road_profile_path)
        
        if self.debug:
            plt.show()
    
        # Apply Savitzky-Golay filter to velocity data
        window_size = 51  # Adjust as needed
        poly_order = 3  # Adjust as needed
        
        velocities_smoothed = savgol_filter(self.velocities, window_size, poly_order)
        
        # Plot velocities
        plt.figure(figsize=(12, 8))
        plt.plot(self.times, velocities_smoothed)
    
        # # Plot velocities
        # plt.figure(figsize=(12, 8))
        # plt.plot(self.times, self.velocities)
        plt.title('Vertical Velocity vs. Time')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Velocitiy (m/s)')
        plt.grid(True, linestyle="--", alpha=0.7)
        
        # Save the velocities plot
        velocities_path = os.path.join(image_dir, 'velocities.png')
        plt.savefig(velocities_path)
        
        if self.debug:
            plt.show()
    
        # Apply Savitzky-Golay filter to both position and velocity data
        window_size = 51  # Adjust as needed
        poly_order = 3  # Adjust as needed
        
        positions_smoothed = savgol_filter(self.positions, window_size, poly_order)
        velocities_smoothed = savgol_filter(self.velocities, window_size, poly_order)
        
        ## Consider scrapping the phase plot code. I can't see any meaningful data from it
        
        # Plot phase space plot
        plt.figure(figsize=(12, 8))
        plt.plot(positions_smoothed, velocities_smoothed, label='Phase Space Plot')
    
        # # Plot phase space plot
        plt.title('Position-Velocity Phase Plot')
        plt.xlabel('Vertical Displacement (meters)')  # Updated xlabel
        plt.ylabel('Velocities')
        plt.grid(True, linestyle="--", alpha=0.7)
        
        # Save the phase space plot
        phase_space_path = os.path.join(image_dir, 'phase_space_plot.png')
        plt.savefig(phase_space_path)
        
        if self.debug:
            plt.show()
        
        # Plot forcing function
        plt.figure(figsize=(12, 8))
        plt.plot(self.times, self.sinusoidal_forcing_function(self.times, *self.fit_params))
        plt.title('Base Excitation from Road Surface')
        plt.xlabel('Time (seconds)')  # Updated xlabel
        plt.ylabel(r'$F(t)$ (Newtons)')  # Updated ylabel to Latex-style
        plt.grid(True, linestyle="--", alpha=0.7)
        
        # Annotate with Latex-style formatted equation and numeric values
        equation = fr'$A \sin(\omega_n t + \phi_0)$'  # Equation template
        numeric_values = fr'$A = {self.fitted_amplitude:.2f}$' \
                          fr'\n$\omega_n = {self.fitted_angular_frequency:.2f}$' \
                          fr'\n$\phi_0 = {self.fitted_phase_shift:.2f}$'
        
        # Combine equation and numeric values
        full_annotation = f'{equation}\n{numeric_values}'
        
        plt.annotate(text=full_annotation,
                     xy=(0.95, 0.95),  # Top-right position
                     xycoords='axes fraction',
                     ha='right',  # Right-align the text
                     va='top',  # Align to the top of the bounding box
                     fontsize=12,
                     bbox=dict(boxstyle='round', fc='w'))
        
        # Save the forcing function plot
        forcing_function_path = os.path.join(image_dir, 'forcing_function.png')
        plt.savefig(forcing_function_path)
        
        if self.debug:
            plt.show()

