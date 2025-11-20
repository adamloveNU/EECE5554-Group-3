#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import struct
import math


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        
        # Declare parameters
        self.declare_parameter('port', '/tmp/vserial4')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'laser_link')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.pi / 180.0)  # 1 degree
        self.declare_parameter('time_increment', 0.0)
        self.declare_parameter('scan_time', 0.1)
        self.declare_parameter('range_min', 0.1)  # meters
        self.declare_parameter('range_max', 10.0)  # meters
        
        # Get parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        scan_topic = self.get_parameter('scan_topic').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.time_increment = self.get_parameter('time_increment').value
        self.scan_time = self.get_parameter('scan_time').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        
        # Initialize serial connection
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f'Connected to LiDAR on {port} at {baudrate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            return
        
        # Create publisher
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, 10)
        
        # Create timer to read data
        self.timer = self.create_timer(0.001, self.read_lidar_data)  # 1ms polling
        
        self.packet_count = 0
        self.error_count = 0
        
        # Buffer for incomplete packets
        self.buffer = bytearray()
    
    def read_lidar_data(self):
        """Read and parse LiDAR data from serial port"""
        try:
            # Read available data
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.read(self.serial_port.in_waiting)
                self.buffer.extend(data)
            
            # Process complete packets from buffer
            while len(self.buffer) >= 5:  # Minimum packet size: header(2) + type(1) + num_points(2)
                # Look for header bytes
                header_idx = -1
                for i in range(len(self.buffer) - 1):
                    if self.buffer[i] == 0xAA and self.buffer[i + 1] == 0x55:
                        header_idx = i
                        break
                
                if header_idx == -1:
                    # No header found, clear buffer except last byte (might be part of header)
                    if len(self.buffer) > 1:
                        self.buffer = self.buffer[-1:]
                    break
                
                # Remove bytes before header
                if header_idx > 0:
                    self.buffer = self.buffer[header_idx:]
                
                # Check if we have enough bytes for header + type + num_points
                if len(self.buffer) < 5:
                    break
                
                # Verify packet type
                if self.buffer[2] != 0x01:
                    # Invalid packet type, skip header and continue
                    self.buffer = self.buffer[2:]
                    self.error_count += 1
                    continue
                
                # Read number of points
                num_points = struct.unpack('<H', self.buffer[3:5])[0]
                
                # Calculate expected packet size
                # Header(2) + Type(1) + NumPoints(2) + Ranges(2*num_points) + Checksum(1)
                expected_size = 5 + (2 * num_points) + 1
                
                # Check if we have complete packet
                if len(self.buffer) < expected_size:
                    # Wait for more data
                    break
                
                # Extract packet
                packet = self.buffer[:expected_size]
                self.buffer = self.buffer[expected_size:]
                
                # Verify checksum
                checksum = packet[-1]
                calc_checksum = sum(packet[:-1]) & 0xFF
                
                if checksum != calc_checksum:
                    self.error_count += 1
                    if self.error_count % 100 == 1:
                        self.get_logger().warn(f'Checksum mismatch (errors: {self.error_count})')
                    continue
                
                # Parse range data
                ranges = []
                for i in range(num_points):
                    range_mm = struct.unpack('<H', packet[5 + i*2:5 + i*2 + 2])[0]
                    
                    # Convert from mm to meters
                    if range_mm == 0xFFFF:
                        # Invalid range
                        ranges.append(float('inf'))
                    else:
                        range_m = range_mm / 1000.0
                        # Clamp to valid range
                        if range_m < self.range_min:
                            ranges.append(float('inf'))
                        elif range_m > self.range_max:
                            ranges.append(float('inf'))
                        else:
                            ranges.append(range_m)
                
                # Create and publish LaserScan message
                scan_msg = LaserScan()
                scan_msg.header.stamp = self.get_clock().now().to_msg()
                scan_msg.header.frame_id = self.frame_id
                
                scan_msg.angle_min = self.angle_min
                scan_msg.angle_max = self.angle_max
                scan_msg.angle_increment = self.angle_increment
                scan_msg.time_increment = self.time_increment
                scan_msg.scan_time = self.scan_time
                scan_msg.range_min = self.range_min
                scan_msg.range_max = self.range_max
                scan_msg.ranges = ranges
                
                # Calculate intensities (simple: inverse of range, normalized)
                intensities = []
                for r in ranges:
                    if math.isfinite(r) and r < self.range_max:
                        intensity = (self.range_max - r) / self.range_max
                    else:
                        intensity = 0.0
                    intensities.append(intensity)
                scan_msg.intensities = intensities
                
                # Publish
                self.scan_pub.publish(scan_msg)
                self.packet_count += 1
                
                if self.packet_count % 100 == 0:
                    self.get_logger().info(
                        f'Published {self.packet_count} scans (errors: {self.error_count})'
                    )
                    
        except Exception as e:
            self.get_logger().error(f'Error reading LiDAR: {e}')
    
    def destroy_node(self):
        """Clean up on shutdown"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()