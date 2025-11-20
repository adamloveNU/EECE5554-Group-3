#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
import math
import time
import os


class FileLidarSimulator(Node):
    def __init__(self):
        super().__init__('file_lidar_simulator')
        
        # Declare parameters
        self.declare_parameter('port', '/tmp/vserial3')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('data_file', '')
        self.declare_parameter('playback_speed', 1.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.pi / 180.0)  # 1 degree
        self.declare_parameter('range_min', 0.1)  # meters
        self.declare_parameter('range_max', 10.0)  # meters
        self.declare_parameter('scan_rate', 10.0)  # Hz
        self.declare_parameter('synthetic_mode', False)  # Generate synthetic data
        
        # Get parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        data_file = self.get_parameter('data_file').value
        self.playback_speed = self.get_parameter('playback_speed').value
        self.loop = self.get_parameter('loop').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.scan_rate = self.get_parameter('scan_rate').value
        self.synthetic_mode = self.get_parameter('synthetic_mode').value
        
        # Open serial port
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f'File LiDAR Simulator connected to {port} at {baudrate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.get_logger().error('Make sure virtual ports are created first!')
            return
        
        # Calculate number of ranges
        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        
        # Load data file if provided
        self.scan_data = []
        self.get_logger().warn("data_file")
        self.get_logger().warn(data_file)
        self.get_logger().warn("--")

        if not self.synthetic_mode and data_file:
            if not os.path.exists(data_file):
                self.get_logger().error(f'Data file not found: {data_file}')
                self.get_logger().warn('Falling back to synthetic mode')
                self.synthetic_mode = True
            else:
                self.load_data_file(data_file)
                if len(self.scan_data) == 0:
                    self.get_logger().warn('No valid data loaded from file, falling back to synthetic mode')
                    self.synthetic_mode = True
        
        if self.synthetic_mode:
            self.get_logger().info('Running in synthetic mode - generating LiDAR scans')
        else:
            self.get_logger().info(f'Loaded {len(self.scan_data)} scan samples from {data_file}')
            self.get_logger().info(f'Playback speed: {self.playback_speed}x, Loop: {self.loop}')
            self.current_index = 0
            self.start_time = self.get_clock().now()
        
        self.get_logger().info(f'LiDAR Simulator initialized')
        self.get_logger().info(f'  Angle range: [{math.degrees(self.angle_min):.1f}°, {math.degrees(self.angle_max):.1f}°]')
        self.get_logger().info(f'  Angle increment: {math.degrees(self.angle_increment):.3f}°')
        self.get_logger().info(f'  Range: [{self.range_min}m, {self.range_max}m]')
        self.get_logger().info(f'  Number of ranges: {self.num_ranges}')
        self.get_logger().info(f'  Scan rate: {self.scan_rate} Hz')
        
        # Scan counter
        self.scan_count = 0
        
        # Create timer to send scans
        timer_period = (1.0 / self.scan_rate) / self.playback_speed
        self.timer = self.create_timer(timer_period, self.send_scan)
    
    def load_data_file(self, filepath):
        """Load LiDAR scan data from file"""
        try:
            with open(filepath, 'r') as f:
                for line in f:
                    line = line.strip()
                    # Skip empty lines and comments
                    if not line or line.startswith('#'):
                        continue
                    
                    # Parse CSV: timestamp,range1,range2,...,rangeN
                    parts = line.split(',')
                    if len(parts) < 2:
                        self.get_logger().warn(f'Invalid line: {line}')
                        continue
                    
                    try:
                        timestamp = float(parts[0])
                        ranges = [float(x) for x in parts[1:]]
                        
                        # Validate and clamp ranges
                        valid_ranges = []
                        for r in ranges:
                            if math.isinf(r) or math.isnan(r):
                                valid_ranges.append(self.range_max + 1.0)  # Invalid range
                            else:
                                # Clamp to valid range
                                r = max(self.range_min, min(self.range_max, r))
                                valid_ranges.append(r)
                        
                        self.scan_data.append({
                            'timestamp': timestamp,
                            'ranges': valid_ranges
                        })
                    except ValueError as e:
                        self.get_logger().warn(f'Could not parse line: {line} - {e}')
                        continue
                        
        except Exception as e:
            self.get_logger().error(f'Failed to load data file: {e}')
    
    def generate_synthetic_scan(self):
        """Generate a synthetic LiDAR scan with some obstacles"""
        ranges = []
        
        # Generate synthetic scan data
        for i in range(self.num_ranges):
            angle = self.angle_min + i * self.angle_increment
            
            # Create some obstacles at different angles
            # Obstacle at 0 degrees (front)
            if abs(angle) < 0.1:
                range_val = 2.0 + 0.5 * math.sin(self.scan_count * 0.1)
            # Obstacle at 90 degrees (right)
            elif abs(angle - math.pi / 2) < 0.1:
                range_val = 1.5 + 0.3 * math.cos(self.scan_count * 0.15)
            # Obstacle at -90 degrees (left)
            elif abs(angle + math.pi / 2) < 0.1:
                range_val = 1.8 + 0.4 * math.sin(self.scan_count * 0.12)
            # Obstacle at 180 degrees (back)
            elif abs(abs(angle) - math.pi) < 0.1:
                range_val = 2.5 + 0.6 * math.cos(self.scan_count * 0.08)
            # Default: gradually increasing range with some noise
            else:
                base_range = self.range_max * 0.8
                noise = 0.1 * math.sin(angle * 3 + self.scan_count * 0.05)
                range_val = base_range + noise
            
            # Clamp to valid range
            range_val = max(self.range_min, min(self.range_max, range_val))
            ranges.append(range_val)
        
        return ranges
    
    def send_scan(self):
        """Send a LiDAR scan packet via serial port"""
        # Get ranges
        if self.synthetic_mode:
            ranges = self.generate_synthetic_scan()
        else:
            # Playback from file
            if len(self.scan_data) == 0:
                return
            
            # Calculate elapsed time
            current_time = self.get_clock().now()
            elapsed_sec = (current_time - self.start_time).nanoseconds / 1e9
            
            # Adjust for playback speed
            playback_time = elapsed_sec * self.playback_speed
            
            # Check if we should send the next sample
            if self.current_index < len(self.scan_data):
                sample = self.scan_data[self.current_index]
                
                if playback_time >= sample['timestamp']:
                    ranges = sample['ranges']
                    self.current_index += 1
                    
                    # Log progress
                    if self.current_index % 100 == 0:
                        self.get_logger().info(
                            f'Sent {self.current_index}/{len(self.scan_data)} scans'
                        )
                else:
                    # Not time yet, reuse previous scan
                    if self.current_index > 0:
                        ranges = self.scan_data[self.current_index - 1]['ranges']
                    else:
                        ranges = self.generate_synthetic_scan()
            else:
                # Reached end of data
                if self.loop:
                    self.get_logger().info('Looping back to start...')
                    self.current_index = 0
                    self.start_time = self.get_clock().now()
                    ranges = self.generate_synthetic_scan() if len(self.scan_data) == 0 else self.scan_data[0]['ranges']
                else:
                    self.get_logger().info('Playback complete.')
                    self.timer.cancel()
                    return
        
        # Send scan packet
        self.send_scan_packet(ranges)
        self.scan_count += 1
        
        # Log progress
        if self.scan_count % 100 == 0:
            self.get_logger().info(f'Sent {self.scan_count} scans')
    
    def send_scan_packet(self, ranges):
        """Send LiDAR scan packet via serial port
        
        Packet format:
        - Header: 0xAA 0x55 (2 bytes)
        - Packet type: 0x01 (1 byte) - scan data
        - Number of points: uint16 (2 bytes)
        - Range data: uint16 array (2 bytes per range, in mm)
        - Checksum: uint8 (1 byte)
        """
        num_points = len(ranges)
        
        # Build packet
        packet = bytearray([0xAA, 0x55])  # Header
        packet.append(0x01)  # Packet type: scan data
        packet.extend(struct.pack('<H', num_points))  # Number of points (little-endian)
        
        # Add range data (convert meters to millimeters, as uint16)
        for r in ranges:
            # Convert to mm and clamp to uint16 range
            range_mm = int(r * 1000.0)
            # Use 0xFFFF for invalid ranges (out of bounds)
            if r > self.range_max or r < self.range_min:
                range_mm = 0xFFFF
            else:
                range_mm = max(0, min(0xFFFE, range_mm))  # Clamp to valid range
            
            packet.extend(struct.pack('<H', range_mm))
        
        # Calculate checksum (sum of all bytes except checksum)
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        
        # Send packet
        try:
            self.serial_port.write(packet)
        except Exception as e:
            self.get_logger().error(f'Error sending scan packet: {e}')
    
    def destroy_node(self):
        """Clean up"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FileLidarSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
