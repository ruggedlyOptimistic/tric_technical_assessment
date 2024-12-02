# -*- coding: utf-8 -*-
"""
Created on Sun Nov 19 15:50:49 2023

@author: Jason
"""

import rosbag, os, time, sys, math
import event, gps_event, gps_data, mode_change_event, gnss_event, nav_status_event, antenna_status_event
import road_profile
import numpy as np
from datetime import datetime
import pytz
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

def extract_sim_parameters(target_bag_file):
    array_size = 0
    robot_start_time = None
    sim_date = None
    
    run_time_estimation_factor = 4.9288e-7     # has units of build_time_minutes per message
    
    try:
        with rosbag.Bag(target_bag_file, 'r') as bag:
            array_size = bag.get_message_count()
            
            for topic, message, timestamp in bag.read_messages():
                timestamp = timestamp.to_nsec()
                
                if str(topic) == '/nav/odom':
                    robot_start_time = timestamp    
                    sim_date = datetime.utcfromtimestamp(timestamp/1e9).strftime("%m_%d_%Y-%H_%M")
                    break
        bag.close()        
                
    except FileNotFoundError:
        print(target_bag_file + ' does not exist!')
        sys.exit(0)
    
    estimated_run_time = array_size * run_time_estimation_factor
    
    return [array_size, robot_start_time, sim_date, estimated_run_time]

def extract_gps_from_bag_message(message):
    # Extract relevant data from the message
    seq = message.header.seq  # Added line for extracting 'seq'
    timestamp = message.header.stamp.to_nsec()
    frame_id = message.header.frame_id
    coordinates = np.array([message.pose.pose.position.x,
                            message.pose.pose.position.y,
                            message.pose.pose.position.z])
    quaternion = np.array([message.pose.pose.orientation.x,
                           message.pose.pose.orientation.y,
                           message.pose.pose.orientation.z,
                           message.pose.pose.orientation.w])
    velocity = np.array([message.twist.twist.linear.x,
                         message.twist.twist.linear.y,
                         message.twist.twist.linear.z])
    omega = np.array([message.twist.twist.angular.x,
                      message.twist.twist.angular.y,
                      message.twist.twist.angular.z])
    
    return gps_data.GPS_Data(seq, timestamp=timestamp, frame_id=frame_id,
                       coordinates=coordinates, quaternion=quaternion,
                       velocity=velocity, omega=omega)

def extract_gnss_params(message):
    # Extract relevant data from the message
    fix_type = message.fix_type
    num_sv = message.num_sv
    sbas_used = message.sbas_used
    dngss_used = message.dngss_used
    
    return [fix_type, num_sv, sbas_used, dngss_used]

def extract_antenna_status_params(message):
    # Extract relevant data from the message
    fix_type = message.fix_type
    gps_tow = message.gps_tow
    heading = message.heading
    heading_uncertainty = message.heading_uncertainty
    rcv_1_valid = message.rcv_1_valid
    rcv_2_valid = message.rcv_2_valid    
    antenna_offsets_valid = message.antenna_offsets_valid
    
    return [fix_type, gps_tow, heading, heading_uncertainty, rcv_1_valid, rcv_2_valid, antenna_offsets_valid]

def extract_nav_status_params(message):
    # Extract relevant data from the message
    filter_state = message.filter_state
    dynamics_mode = message.dynamics_mode
    status_flags = message.status_flags
    
    return [filter_state, dynamics_mode, status_flags]  

def format_unix_time(unix_timestamp, timezone='UTC'):
    # Convert UNIX timestamp to a datetime object in UTC
    utc_dt = datetime.fromtimestamp(unix_timestamp / 1e9)  # Convert nanoseconds to seconds
    utc_dt = utc_dt.replace(tzinfo=pytz.utc)

    # Convert to the specified timezone
    target_timezone = pytz.timezone(timezone)
    localized_dt = utc_dt.astimezone(target_timezone)
    
    # Format the datetime object as mm-dd-yyyy HH:mm:ss (24-hour format)
    return localized_dt.strftime("%m-%d-%Y %H:%M:%S")

def time_diff(start_time, stop_time):
    # Calculate and print total robot operation time, given a time in nanoseconds
    total_time_seconds = (stop_time - start_time) / 1e9
    hours, remainder = divmod(total_time_seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    
    return (hours, minutes, seconds)

def extract_gps_velocity_data(event_dict, filter_data=True):
    timestamps = []
    velocities = []

    for timestamp, events in event_dict.items():
        for current_event in events:
            if isinstance(current_event, gps_event.GPS_Event):
                velocities.append(current_event.get_data().get_velocity())
                timestamps.append(timestamp)
                
    if filter_data:
        # Apply Savitzky-Golay filter to velocity data
        window_size = 51  # Adjust as needed
        poly_order = 3  # Adjust as needed    
        
        velocities = savgol_filter(velocities, window_size, poly_order)

    return np.array(timestamps), np.array(velocities)

def plot_gps_position_over_time(timestamps, positions, output_dir, output_filename, axis_specifier='all'):
    # Convert timestamps to seconds from the beginning of the sim
    timestamps_seconds = (timestamps - timestamps[0]) / 1e9
    timestamps_minutes = timestamps_seconds / 60

    # Plot GPS position over time
    plt.figure(figsize=(10, 6))
    
    if axis_specifier == 'all':
        # Use a line instead of the dot + line for GPS position curve
        plt.plot(timestamps_minutes, positions[:, 0], label='Latitude, degrees', linestyle='-', color='b')
        plt.plot(timestamps_minutes, positions[:, 1], label='Longitude, degrees', linestyle='-', color='g')
        plt.plot(timestamps_minutes, positions[:, 2], label='Elevation, meters', linestyle='-', color='r')

    elif axis_specifier == 'x':
        plt.plot(timestamps_minutes, positions[:, 0], label=r'Latitude, degrees', linestyle='-')
    
    elif axis_specifier == 'y':
        plt.plot(timestamps_minutes, positions[:, 1], label=r'Longitude, degrees', linestyle='-')
    
    elif axis_specifier == 'z':
        plt.plot(timestamps_minutes, positions[:, 2], label=r'Elevation, meters', linestyle='-')        
    
    plt.title('GPS Position Over Time')
    plt.xlabel('Time (minutes)')
    plt.ylabel('Position (meters)')
    
    # Annotate the legend with appropriate labels
    plt.legend(loc='upper left')

    plt.grid(True)

    # Check if the output directory exists, create it if not
    os.makedirs(output_dir, exist_ok=True)

    # Save the plot as a PNG file
    output_path = os.path.join(output_dir, output_filename)
    plt.savefig(output_path)

    # Show the plot (optional)
    if debug:
        plt.show()  

def plot_gps_velocity_over_time(timestamps, velocities, output_dir, output_filename='gps_velocity_plot.png'):
    # need to alter the way that timestamps are being processed since they are np.array() type
    
    
    
    # Convert timestamps to seconds from the beginning of the sim
    timestamps_seconds = (timestamps - timestamps[0]) / 1e9
    timestamps_minutes = (timestamps_seconds / 60)

    # Plot GPS velocity over time
    plt.figure(figsize=(10, 6))
    
    # Use a line instead of the dot + line for GPS velocity curve
    plt.plot(timestamps_minutes, velocities[:, 0], label='X Velocity', linestyle='-', color='b')
    plt.plot(timestamps_minutes, velocities[:, 1], label='Y Velocity', linestyle='-', color='g')
    plt.plot(timestamps_minutes, velocities[:, 2], label='Z Velocity', linestyle='-', color='r')
    
    plt.title('GPS Velocity Over Time')
    plt.xlabel('Time (minutes)')
    plt.ylabel('Velocity (m/s)')
    
    # Annotate the legend with appropriate labels
    plt.legend(loc='upper left')

    plt.grid(True)

    # Check if the output directory exists, create it if not
    os.makedirs(output_dir, exist_ok=True)

    # Save the plot as a PNG file
    output_path = os.path.join(output_dir, output_filename)
    plt.savefig(output_path)

    # Show the plot (optional)
    if debug:
        plt.show()
    
def plot_birds_eye_view(timestamps, positions, angles, output_dir, output_filename='2D_position.png'):   
    R = 6371000 # radius of the earth in meters
    z_offset = 2.05 # meters
    
    num_rows = angles.shape[0]  # recall numpy_array.shape() returns the dimensions of the array.
                                # shape[0] returns the number of rows
    offsets = np.zeros((num_rows, 3))    
    
    # Extract X and Y coordinates from positions
    x_coordinates = R * (positions[:, 0] - positions[0, 0])  # Shift X coordinates to start from zero
    y_coordinates = R * (positions[:, 1] - positions[0, 1])  # Shift Y coordinates to start from zero

    for i in range(0,num_rows):
        offsets[i, 0] = z_offset * math.sin(angles[i, 1])
        offsets[i, 1] = z_offset * math.sin(angles[i, 0])
        # offsets[i, 2] = -z_offset

    # Calculate corrected coordinates
    corrected_x_coordinates = x_coordinates + offsets[:, 0]
    corrected_y_coordinates = y_coordinates + offsets[:, 1]

    # Plot 2D bird's eye view
    plt.figure(figsize=(10,10))
    plt.plot(x_coordinates, y_coordinates, label='Original Position', linestyle='-', color='b')  # Original position
    plt.plot(corrected_x_coordinates, corrected_y_coordinates, label='Corrected Position', linestyle=':', color='r')  # Corrected position
    plt.title('Robot Position (2D Bird\'s Eye View)')
    plt.xlabel('X Displacement (meters)')  # Change xlabel
    plt.ylabel('Y Displacement (meters)')  # Change ylabel
    plt.legend()
    plt.grid(True)

    # Check if the output directory exists, create it if not
    os.makedirs(output_dir, exist_ok=True)

    # Save the plot as a JPG file with a resolution of 800x800 pixels
    output_path = os.path.join(output_dir, output_filename)
    plt.savefig(output_path, dpi=100, bbox_inches='tight')  # Set dpi for resolution and bbox_inches for tight layout

    # Save the plot as a PNG file
    output_path = os.path.join(output_dir, output_filename)
    plt.savefig(output_path)

    # Show the plot (optional)
    if debug:    
        plt.show()

def generate_report(robot_start_time, robot_stop_time, output_dir, output_file):
    # making the new directory if it does not already exist
    os.makedirs(output_dir, exist_ok=True)
    f_out = os.path.join(output_dir,output_file)
    
    with open(f_out, 'w') as report_file:
        # Convert and format time in PST (Pacific Standard Time)
        report_file.write('Robot Start Time: \t\t\t' + format_unix_time(robot_start_time, 'America/Los_Angeles') + '\n')
        report_file.write('Robot Stop Time: \t\t\t' + format_unix_time(robot_stop_time, 'America/Los_Angeles') + '\n')

        robot_run_time = time_diff(robot_start_time, robot_stop_time)
        report_file.write(f"Total Robot Operation Time:  \t\t\t{int(robot_run_time[0])}:{int(robot_run_time[1])}:{robot_run_time[2]:.2f}\n")

        report_file.write('\n*******************************************************************************\n')
        
# GLOBAL VARIABLE FOR DEBUGGING ONLY
debug = False

if __name__ == '__main__':
    
    #from std_msgs.msg import Int32, String
    target_bag_file = 'test_bag.bag'
    
    print('Computing initial sim parameters...',end='')
    initial = time.time()
    
    sim_params = extract_sim_parameters(target_bag_file)
    
    array_size = sim_params[0]
    
    # tracking the robot run time:
    robot_start_time = sim_params[1]
    robot_stop_time = None
    
    sim_date = sim_params[2]
    estimated_run_time_minutes = sim_params[3]
    
    print('done.\t\t\t\t\t\t[{} seconds]'.format(str(round(time.time() - initial, 3))))

    # this dictionary keeps track of our events, which are derived from the hardcoded list of important topics
    event_dict = {}
    
    # naming the simulation output directory
    output_dir = 'simout_' + str(sim_date)
    # making the new directory if it does not already exist
    os.makedirs(output_dir, exist_ok=True)
    
    # making a subdirectory for images
    image_dir = os.path.join(output_dir,'images')
    os.makedirs(image_dir, exist_ok = True)
    # currently not doing anything with this variable
    gps_data_file = os.path.join(output_dir,'gps_data_' + str(sim_date) + '.csv')
    
    master_event_report = os.path.join(output_dir,'master_event_report_' + str(sim_date) + '.csv')
    
    # modify this list to change the topics to analyze
    # we should load this from a file * Later
    important_topics = ['/nav/odom', '/drive_mode', '/control_mode', '/gnss1/fix_info', '/gnss2/fix_info', '/nav/dual_antenna_status', '/nav/status']
    
    # start the timer right before we start the simulation
    start = time.time()
    print('Reading from bag file...', end='')
    
    try:
        
        # open the rosbag and attempt to read
        with rosbag.Bag(target_bag_file, 'r') as bag, open(gps_data_file, 'w') as gps_file:            
            print('{} messages found. \nBuilding event queue...estimated simulation build time:\t\t[{} minutes]'.format(str(array_size), str(round(estimated_run_time_minutes, 3))))
            print('*******************************************************************************\n')
                
            # writing header for gps_file
            gps_header = 'Timestamp_UNIX,Timestamp_readable,Topic,Sensor_name,Latitude,Longtude,Elevation,Velocity_x,Velocity_y,Velocity_z,Roll,Pitch,Yaw,Roll_Rate,Pitch_Rate,Yaw_Rate\n'
            gps_file.write(gps_header)
            
            for topic, message, timestamp in bag.read_messages():
                # converting the ROS timestamp to float
                timestamp = timestamp.to_nsec()
                
                if str(topic) in important_topics:
                    # filter objects
                    
                    if str(topic) == '/nav/odom':
                        # Create GPS_Data object
                        data = extract_gps_from_bag_message(message)
                        e = gps_event.GPS_Event(str(topic), 'nav_msgs/Odometry', timestamp, data)
                            
                        # Assuming the last odom timestamp is the robot stop time
                        robot_stop_time = timestamp
                        
                        coordinates = e.get_data().get_coordinates()
                        xd = e.get_data().get_velocity()
                        angles = e.get_data().get_angles()
                        omega = e.get_data().get_omega()
                        
                        line = '{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(
                            timestamp,format_unix_time(timestamp),
                            e.get_topic(),e.get_data().get_sensor_name(),
                            coordinates[0],coordinates[1],coordinates[2],
                            xd[0], xd[1], xd[2],
                            angles[0], angles[1], angles[2],
                            omega[0], omega[1], omega[2])
                        gps_file.write(line)
                        
                    elif str(topic) == '/drive_mode' or str(topic) == '/control_mode':
                        # Using 'None' as a data type will probably come back to haunt me later...
                        e = mode_change_event.Mode_Change_Event(str(topic), None, timestamp, mode=message.data)
                        
                    elif str(topic) == '/gnss1/fix_info' or str(topic) == '/gnss2/fix_info':
                        
                        params = extract_gnss_params(message)
                        e = gnss_event.GNSS_Event(str(topic), None, timestamp, params[0], params[1], params[2], params[3])
                        
                    elif str(topic) == '/nav/dual_antenna_status':
                        params = extract_antenna_status_params(message)
                        e = antenna_status_event.Antenna_Status_Event(str(topic), None, timestamp, params[0], params[1], params[2], params[3], params[4], params[5], params[6])
                    
                    elif str(topic) == '/nav/status':
                        
                        params = extract_nav_status_params(message)
                        e = nav_status_event.Nav_Status_Event(str(topic), None, timestamp, params[0], params[1], params[2])
                    
                    # Check if the timestamp is already in the dictionary
                    if timestamp not in event_dict:   
                        # If it is not, create a new list associated with that timestamp
                        event_dict[timestamp] = [e]
                    else:
                        # If it already exists, append the event to the list
                        event_dict[timestamp].append(e)
                        
            print('Event queue populated in ' + str(round(time.time() - start, 3)) + ' seconds with ' + str(len(event_dict)) + ' entries')            
    
        bag.close()
        gps_file.close()
            
    except FileNotFoundError:
        print('Unable to open %s\n', target_bag_file)
        exit(0)    
    
    print('Generating master event report...',end='')
    try:
       
        with open(master_event_report, 'w') as event_report:
            
            # write the file header
            event_report.write('Timestamp_UNIX,Latitude,Longitude,Elevation,Velocity_x,Velocity_y,Velocity_z,Roll,Pitch,Yaw,Roll_Rate,Pitch_Rate,Yaw_Rate,master_drive_mode,master_control_state,gnss1_fix_info,gnss2_fix_info,nav/dual_antenna_status/msg.fix_type,nav/dual_antenna_status/msg.antenna_offsets_valid,nav/status/msg.filter_state\n')
            
            # Create numpy arrays
            num_rows = len(event_dict.items())
          
            
            array_index = 0
            
            times = np.zeros((num_rows, 1))
            positions = np.zeros((num_rows, 3))
            velocities = np.zeros((num_rows, 3))
            angles = np.zeros((num_rows, 3))
          
            offsets = np.zeros((num_rows, 3))
            
            # using this as a way to reduce the number of un-initialized values for our simulation
            prev_entry = [-1 for i in range(0,20)]
            
            # iterating over each list of events at each timestamp
            for timestamp, events in event_dict.items():
                # create a list with 20 placeholders
                
                entry = [-1 for i in range(0,20)]
                    
                # setting the timestamp
                entry[0] = timestamp
                
                # iterating over all of the events in the dictionary at each timestamp to scrape data
                for this_event in events:
                    
                    if this_event.get_topic() == '/nav/odom':
                        # Create GPS_Data object
                        # data = this_event.get_data()
                        
                        # Extracting gps data
                        coordinates = this_event.get_data().get_coordinates()
                        xd = this_event.get_data().get_velocity()
                        th = this_event.get_data().get_angles()
                        thd = this_event.get_data().get_omega()
                        
                        entry[1] = coordinates[0]   #latitude
                        entry[2] = coordinates[1]   #longitude
                        entry[3] = coordinates[2]   #elevation
                        entry[4] = xd[0]            #velocity_x
                        entry[5] = xd[1]            #velocity_y
                        entry[6] = xd[2]            #velocity_z
                        entry[7] = th[0]            #roll_angle
                        entry[8] = th[1]            #pitch_angle
                        entry[9] = th[2]            #yaw_angle
                        entry[10] = thd[0]          #roll_rate
                        entry[11] = thd[1]          #pitch_rate
                        entry[12] = thd[2]          #yaw_raw
                        
                    elif this_event.get_topic() == '/drive_mode':
                        entry[13] = this_event.get_mode()                      #master_drive_mode
                    
                    elif this_event.get_topic() == '/control_mode':
                        entry[14] = this_event.get_mode()                      #master_control_mode
                        
                    elif this_event.get_topic() == '/gnss1/fix_info':
                        entry[15] = this_event.get_fix_type()                  #gnss1_fix_type
                        
                    elif this_event.get_topic() == '/gnss2/fix_info':
                        entry[16] = this_event.get_fix_type()                  #gnss2_fix_type
                        
                    elif this_event.get_topic() == '/nav/dual_antenna_status':
                        entry[17] = this_event.get_fix_type()                  #nav/dual_antenna_status/msg.fix_type
                        entry[18] = this_event.get_antenna_offsets_valid()     #nav/dual_antenna_status/msg.antenna_offsets_valid

                    elif this_event.get_topic() == '/nav/status':
                        entry[19] = this_event.get_filter_state()              #nav/status/msg.filter_state 
                
                for i in range(0,20):
                    # if the current entry value is not set, check the previous entry value
                    if entry[i] == -1 and prev_entry[i] != -1:
                        entry[i] = prev_entry[i]
                
                # Flag to start populating the numpy arrays
                good_data = True
                # TODO
                # can change this based on the sensor inputs
                for i in range(1,20):
                    
                    # Note: this does NOT effect the entries, only the stuff that gets plotted
                    if entry[i] == -1 or entry[15] != 6 or entry[16] != 6 or entry[17] != 2 or entry[18] != 4 or entry[19] != 4 or (entry[4] == 0 and entry[5] == 0 and entry[6] == 0):
                        good_data = False
                        break
                # build numpy arrays here before closing the file
                if good_data:
                    
                    # Buliding arrays associated with good data
                    times[array_index] = entry[0]           # Timestamp
                    positions[array_index, 0] = entry[1]   # Latitude
                    positions[array_index, 1] = entry[2]   # Longitude
                    positions[array_index, 2] = entry[3]   # Elevation
    
                    velocities[array_index, 0] = entry[4]  # Velocity_x
                    velocities[array_index, 1] = entry[5]  # Velocity_y
                    velocities[array_index, 2] = entry[6]  # Velocity_z
            
                    angles[array_index, 0] = entry[7]  # Roll
                    angles[array_index, 1] = entry[8]  # Pitch
                    angles[array_index, 2] = entry[9]  # Yaw
                    
                    #only increment the index when good data is found
                    array_index +=1
        
                # updating the value stored in 'prev_entry'   
                prev_entry = entry
                
                #writing the entry to the next line in the report
                event_report.write('{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(entry[0],entry[1],entry[2],entry[3],entry[4],
                                                                                                          entry[5],entry[6],entry[7],entry[8],entry[9],
                                                                                                          entry[10],entry[11],entry[12],entry[13],entry[14],
                                                                                                          entry[15],entry[16],entry[17],entry[18],entry[19]))                                                                                   
            
            
            # Finding the first zero index
            first_zero_index = np.where(np.all(positions == 0, axis=1))[0][0]
            
            # Truncate all arrays from the first zero index to the end
            times = times[:first_zero_index]
            positions = positions[:first_zero_index, :]
            velocities = velocities[:first_zero_index, :]
            angles = angles[:first_zero_index, :]
            
            if debug:
            
                print('Writing plot data...',end='')
                with open('plot_data.csv', 'w') as plot_data:
                    plot_data.write('time_seconds,x,y,z,xd,yd,zd\n')
                    
                    for i in range(0,times.shape[0]):
                        
                        t = str(float((times[i] - times[0])/1e9))
                        x = str(float(positions[i,0]))
                        y = str(float(positions[i,1]))
                        z = str(float(positions[i,2]))
                        xd = str(float(velocities[i,0]))
                        yd = str(float(velocities[i,1]))
                        zd = str(float(velocities[i,2]))
                        
                        plot_data.write('{},{},{},{},{},{},{}\n'.format(t,x,y,z,xd,yd,zd))
                        
                plot_data.close()
                print('done')
            
            print('Generating media...',end='')
            # ************* Start Data Visualization Tasks ***************
            # Plot GPS velocity over time
            plot_gps_velocity_over_time(times, velocities, image_dir)
            # Plot 2D bird's eye view
            plot_birds_eye_view(times, positions, angles, image_dir)
            # Subplots of x and y displacements over time
            plot_gps_position_over_time(times, positions, image_dir, 'x_displacement.png', 'x')
            plot_gps_position_over_time(times, positions, image_dir, 'y_displacement.png','y')
            # Generate road profile
            profile = road_profile.Road_Profile((times - times[0])/1e9, positions[:,2], velocities[:,2], 2000)           # assuming robot mass of 2000 kg
            profile.plot_results(image_dir)
            
                                                                                                          
        event_report.close()
        print('done')
                
    except FileNotFoundError:
        print("Could not write the master event report!")
        pass          
    
    # Print summary to the terminal
    print('\n***************************** Simulation Summary ******************************\n')
    # Convert and format time in PST (Pacific Standard Time)
    print('Robot Start Time: \t\t\t' + format_unix_time(robot_start_time, 'America/Los_Angeles'))
    print('Robot Stop Time: \t\t\t' + format_unix_time(robot_stop_time, 'America/Los_Angeles'))
    
    robot_run_time = time_diff(robot_start_time, robot_stop_time)
    print(f"Total Robot Operation Time:  \t\t\t{int(robot_run_time[0])}:{int(robot_run_time[1])}:{robot_run_time[2]:.2f}")
    
    stop = time.time()
    print('\n*******************************************************************************')
    print('Total simulation build time: ' + str(round(stop - start, 3)) + ' seconds')
    
    generate_report(robot_start_time, robot_stop_time, output_dir, 'op_summary.txt')