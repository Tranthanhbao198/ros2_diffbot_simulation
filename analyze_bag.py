#!/usr/bin/env python3
#source /opt/ros/humble/setup.bash first
import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

# ================================================================
# UTILITY FUNCTIONS (UNCHANGED)
# ================================================================

def read_bag_to_dataframe(bag_folder_path, topic_name):
    """
    Reads data from a specific topic in a ROS2 bag file into a Pandas DataFrame.
    """
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    except ImportError as e:
        print("\n================================================================================")
        print(f"ERROR: Could not import rosbag2_py.")
        print("Please make sure you have sourced your ROS 2 environment before running this script:")
        print("source /opt/ros/humble/setup.bash")
        print(f"Error details: {e}")
        print("================================================================================\n")
        return None

    storage_options = StorageOptions(uri=bag_folder_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag file at '{bag_folder_path}': {e}")
        return None

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_type.name: topic_type.type for topic_type in topic_types}

    if topic_name not in type_map:
        print(f"Error: Topic '{topic_name}' not found in the bag file.")
        print(f"Available topics: {[t.name for t in topic_types]}")
        return None

    msg_type = get_message(type_map[topic_name])
    data = []
    while reader.has_next():
        (topic, msg_data, timestamp_ns) = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(msg_data, msg_type)
            timestamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            velocity_x = msg.twist.twist.linear.x
            data.append({'timestamp': timestamp_sec, 'velocity_x': velocity_x})
    
    return pd.DataFrame(data)

def find_events(df, target_velocity=5.0, velocity_threshold=0.01):
    """
    Finds key event timestamps in the performance data DataFrame.
    """
    events = {
        'start_accel': None,
        'reach_max_vel': None,
        'start_decel': None,
        'stop': None
    }
    
    start_accel_idx = df[df['velocity_x'] > velocity_threshold].first_valid_index()
    if start_accel_idx is not None:
        events['start_accel'] = df.loc[start_accel_idx]

    reach_max_vel_idx = df[df['velocity_x'] >= (target_velocity * 0.99)].first_valid_index()
    if reach_max_vel_idx is not None:
        events['reach_max_vel'] = df.loc[reach_max_vel_idx]
    
    if reach_max_vel_idx is not None:
        start_decel_idx = df.loc[reach_max_vel_idx:][df['velocity_x'] < (target_velocity * 0.99)].first_valid_index()
        if start_decel_idx is not None:
            events['start_decel'] = df.loc[start_decel_idx - 1]

    if start_decel_idx is not None:
        stop_idx = df.loc[start_decel_idx:][df['velocity_x'] <= velocity_threshold].first_valid_index()
        if stop_idx is not None:
            events['stop'] = df.loc[stop_idx]
            
    return events

# ================================================================
# MAIN SCRIPT EXECUTION
# ================================================================
if __name__ == '__main__':
    
    bag_folder = os.path.expanduser('~/ros2_ws/logfile') 
    topic_to_analyze = '/odom'

    print(f"Starting analysis of bag file in directory: {bag_folder}")
    
    df = read_bag_to_dataframe(bag_folder, topic_to_analyze)
    
    if df is not None and not df.empty:
        print(f"Successfully read {len(df)} messages.")
        
        df['time_rel'] = df['timestamp'] - df['timestamp'].iloc[0]
        events = find_events(df, target_velocity=5.0, velocity_threshold=0.05)
        
        # Print the analysis report to the terminal (unchanged)
        print("\n================ PERFORMANCE ANALYSIS ================")
        # ... (rest of the print statements)
        if events['start_accel'] is not None:
            print(f"- Acceleration started at: {events['start_accel']['time_rel']:.3f} s")
        if events['reach_max_vel'] is not None:
            print(f"- Max velocity reached at: {events['reach_max_vel']['time_rel']:.3f} s")
        if events['start_decel'] is not None:
            print(f"- Deceleration started at: {events['start_decel']['time_rel']:.3f} s")
        if events['stop'] is not None:
            print(f"- Full stop at:            {events['stop']['time_rel']:.3f} s")
        
        print("--------------------------------------------------")
        
        if events['start_accel'] is not None and events['reach_max_vel'] is not None:
            accel_time = events['reach_max_vel']['time_rel'] - events['start_accel']['time_rel']
            print(f"=> Acceleration Time (0 -> 5m/s): {accel_time:.3f} s (Requirement: ~5s)")
        
        if events['start_decel'] is not None and events['stop'] is not None:
            decel_time = events['stop']['time_rel'] - events['start_decel']['time_rel']
            print(f"=> Deceleration Time (5m/s -> 0): {decel_time:.3f} s (Requirement: ~1s)")
        print("==================================================")


        # ================================================================
        # PROFESSIONAL PLOTTING WITH A CUSTOM LEGEND
        # ================================================================
        fig, ax = plt.subplots(figsize=(15, 8))
        ax.plot(df['time_rel'], df['velocity_x'], label='Robot Velocity', color='dodgerblue', linewidth=2.5)

        # Define styles for each event marker
        event_styles = {
            'start_accel':   {'color': 'green',       'name': 'Start Acceleration'},
            'reach_max_vel': {'color': 'purple',      'name': 'Reach Max Velocity'},
            'start_decel':   {'color': 'darkorange',  'name': 'Start Deceleration'},
            'stop':          {'color': 'red',         'name': 'Full Stop'}
        }
        
        legend_handles = []
        # Plot markers on the graph and create handles for the custom legend
        for event_name, event_data in events.items():
            if event_data is not None:
                time_point = event_data['time_rel']
                vel_point = event_data['velocity_x']
                style = event_styles[event_name]
                
                # Plot the colored marker on the curve
                ax.plot(time_point, vel_point, 'o', markersize=9, color=style['color'], markeredgecolor='black')
                
                # Create a custom legend entry (handle) for this event
                # This is a "dummy" plot element that won't be visible but will appear in the legend
                legend_label = f"{style['name']:<22}: {time_point:.3f} s"
                handle = Line2D([0], [0], marker='o', color='w', label=legend_label,
                                markerfacecolor=style['color'], markersize=10)
                legend_handles.append(handle)

        # Configure plot aesthetics
        ax.set_title('Robot Acceleration and Deceleration Performance', fontsize=18, fontweight='bold')
        ax.set_xlabel('Time (seconds)', fontsize=14)
        ax.set_ylabel('Linear Velocity (m/s)', fontsize=14)
        ax.grid(True, which='both', linestyle=':')
        ax.set_ylim(-0.5, 6.5)
        
        # Create the final, table-like legend from our custom handles
        if legend_handles:
            ax.legend(handles=legend_handles, title="Key Event Timestamps", 
                      fontsize=11, title_fontsize=13, fancybox=True, framealpha=0.8)

        plt.tight_layout()
        plt.show()
        
    else:
        print("No data to analyze or an error occurred.")
