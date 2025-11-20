#!/usr/bin/env python3
import sqlite3
import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def extract_lidar_from_ros2_bag(bag_path, topic_name, output_file):
    """Extract LiDAR data from ROS2 bag to text file"""
    
    # Connect to bag database
    db_path = bag_path + '/b0-2014-07-21-12-55-35.db3'
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Get topic id
    cursor.execute("SELECT id, type FROM topics WHERE name=?", (topic_name,))
    topic_result = cursor.fetchone()
    
    if not topic_result:
        print(f"Topic {topic_name} not found in bag!")
        cursor.execute("SELECT name FROM topics")
        topics = cursor.fetchall()
        print(f"Available topics: {[t[0] for t in topics]}")
        conn.close()
        return
    
    topic_id, msg_type = topic_result
    print(f"Found topic: {topic_name}")
    print(f"Message type: {msg_type}")
    
    # Get message class
    msg_class = get_message(msg_type)
    
    # Query messages
    cursor.execute("""
        SELECT timestamp, data 
        FROM messages 
        WHERE topic_id=? 
        ORDER BY timestamp
    """, (topic_id,))
    
    messages = cursor.fetchall()
    print(f"Found {len(messages)} messages")
    
    if len(messages) == 0:
        conn.close()
        return
    
    # Get first timestamp
    first_timestamp = messages[0][0]
    
    # Write to output file
    with open(output_file, 'w') as f:
        f.write("# Format: timestamp,num_points,range1,range2,...,rangeN\n")
        f.write("# Ranges in meters (MultiEchoLaserScan - using first echo)\n")
        
        for timestamp, data in messages:
            # Deserialize
            msg = deserialize_message(data, msg_class)
            
            # Relative timestamp in seconds
            time_sec = (timestamp - first_timestamp) / 1e9
            
            # MultiEchoLaserScan has multiple echoes per measurement
            # We'll use the first echo (most common)
            if len(msg.ranges) > 0 and len(msg.ranges[0].echoes) > 0:
                # Extract first echo from each range measurement
                ranges = [r.echoes[0] if len(r.echoes) > 0 else float('inf') for r in msg.ranges]
            else:
                # Fallback: treat as regular scan
                ranges = msg.ranges
            
            num_points = len(ranges)
            ranges_str = ','.join([f"{r:.3f}" for r in ranges])
            
            # Write
            f.write(f"{time_sec:.3f},{num_points},{ranges_str}\n")
    
    conn.close()
    print(f"LiDAR data written to: {output_file}")
    print(f"Duration: {(messages[-1][0] - first_timestamp) / 1e9:.2f} seconds")
    print(f"Number of scans: {len(messages)}")

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: python3 ros2_bag_to_lidar_txt.py <bag_folder> <topic_name> <output_file>")
        print("Example: python3 ros2_bag_to_lidar_txt.py ~/final-project/b0-2014-07-21-12-55-35 horizontal_laser_2d lidar_data.txt")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    topic_name = sys.argv[2]
    output_file = sys.argv[3]
    
    extract_lidar_from_ros2_bag(bag_path, topic_name, output_file)