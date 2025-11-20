#!/usr/bin/env python3
import sqlite3
import math
import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to euler angles"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

def extract_imu_from_ros2_bag(bag_path, topic_name, output_file):
    """Extract IMU data from ROS2 bag to text file"""
    
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
        f.write("# Format: timestamp,roll,pitch,yaw,wx,wy,wz,ax,ay,az\n")
        f.write("# All angles in radians, angular velocities in rad/s, accelerations in m/s^2\n")
        
        for timestamp, data in messages:
            # Deserialize
            msg = deserialize_message(data, msg_class)
            
            # Relative timestamp in seconds
            time_sec = (timestamp - first_timestamp) / 1e9
            
            # Extract orientation
            roll, pitch, yaw = quaternion_to_euler(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            )
            
            # Extract angular velocity
            wx = msg.angular_velocity.x
            wy = msg.angular_velocity.y
            wz = msg.angular_velocity.z
            
            # Extract acceleration
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            
            # Write
            f.write(f"{time_sec:.3f},{roll:.6f},{pitch:.6f},{yaw:.6f},"
                   f"{wx:.6f},{wy:.6f},{wz:.6f},"
                   f"{ax:.6f},{ay:.6f},{az:.6f}\n")
    
    conn.close()
    print(f"IMU data written to: {output_file}")
    print(f"Duration: {(messages[-1][0] - first_timestamp) / 1e9:.2f} seconds")

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: python3 ros2_bag_to_imu_txt.py <bag_folder> <topic_name> <output_file>")
        print("Example: python3 ros2_bag_to_imu_txt.py ~/final-project/b0-2014-07-21-12-55-35 imu imu_data.txt")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    topic_name = sys.argv[2]
    output_file = sys.argv[3]
    
    extract_imu_from_ros2_bag(bag_path, topic_name, output_file)