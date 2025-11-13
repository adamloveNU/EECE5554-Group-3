#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
import math
import time
import os

class FileIMUSimulator(Node):
    def __init__(self):
        super().__init__('file_imu_simulator')
        
        # Declare parameters
        self.declare_parameter('port', '/tmp/vserial1')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('data_file', '')
        self.declare_parameter('playback_speed', 1.0)
        self.declare_parameter('loop', True)
        
        # Get parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        data_file = self.get_parameter('data_file').value
        self.playback_speed = self.get_parameter('playback_speed').value
        self.loop = self.get_parameter('loop').value
        
        # Open serial port
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f'File IMU Simulator connected to {port} at {baudrate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.get_logger().error('Make sure virtual ports are created first!')
            return
        
        # Load data file
        self.imu_data = []
        if not data_file:
            self.get_logger().error('No data file specified!')
            return
            
        if not os.path.exists(data_file):
            self.get_logger().error(f'Data file not found: {data_file}')
            return
            
        self.load_data_file(data_file)
        
        if len(self.imu_data) == 0:
            self.get_logger().error('No valid data loaded from file!')
            return
        
        self.get_logger().info(f'Loaded {len(self.imu_data)} IMU samples from {data_file}')
        self.get_logger().info(f'Playback speed: {self.playback_speed}x, Loop: {self.loop}')
        
        # Playback state
        self.current_index = 0
        self.start_time = self.get_clock().now()
        
        # Create timer - check every 1ms
        self.timer = self.create_timer(0.001, self.playback_data)
        
    def load_data_file(self, filepath):
        """Load IMU data from CSV file"""
        try:
            with open(filepath, 'r') as f:
                for line in f:
                    line = line.strip()
                    # Skip empty lines and comments
                    if not line or line.startswith('#'):
                        continue
                    
                    # Parse CSV: timestamp,roll,pitch,yaw,wx,wy,wz,ax,ay,az
                    parts = line.split(',')
                    if len(parts) != 10:
                        self.get_logger().warn(f'Invalid line: {line}')
                        continue
                    
                    try:
                        timestamp = float(parts[0])
                        roll = float(parts[1])
                        pitch = float(parts[2])
                        yaw = float(parts[3])
                        wx = float(parts[4])
                        wy = float(parts[5])
                        wz = float(parts[6])
                        ax = float(parts[7])
                        ay = float(parts[8])
                        az = float(parts[9])
                        
                        self.imu_data.append({
                            'timestamp': timestamp,
                            'roll': roll,
                            'pitch': pitch,
                            'yaw': yaw,
                            'wx': wx,
                            'wy': wy,
                            'wz': wz,
                            'ax': ax,
                            'ay': ay,
                            'az': az
                        })
                    except ValueError as e:
                        self.get_logger().warn(f'Could not parse line: {line} - {e}')
                        continue
                        
        except Exception as e:
            self.get_logger().error(f'Failed to load data file: {e}')
    
    def playback_data(self):
        """Send IMU data from file at specified playback speed"""
        if len(self.imu_data) == 0:
            return
        
        # Calculate elapsed time
        current_time = self.get_clock().now()
        elapsed_sec = (current_time - self.start_time).nanoseconds / 1e9
        
        # Adjust for playback speed
        playback_time = elapsed_sec * self.playback_speed
        
        # Check if we should send the next sample
        if self.current_index < len(self.imu_data):
            sample = self.imu_data[self.current_index]
            
            if playback_time >= sample['timestamp']:
                # Send this sample
                self.send_imu_packets(sample)
                self.current_index += 1
                
                # Log progress
                if self.current_index % 100 == 0:
                    self.get_logger().info(
                        f'Sent {self.current_index}/{len(self.imu_data)} samples'
                    )
        else:
            # Reached end of data
            if self.loop:
                self.get_logger().info('Looping back to start...')
                self.current_index = 0
                self.start_time = self.get_clock().now()
            else:
                self.get_logger().info('Playback complete.')
                self.timer.cancel()
    
    def send_imu_packets(self, sample):
        """Send WT60 protocol packets for a single IMU sample"""
        # Send acceleration packet (0x51)
        self.send_acceleration_packet(sample['ax'], sample['ay'], sample['az'])
        time.sleep(0.001)  # Small delay
        
        # Send angular velocity packet (0x52)
        self.send_angular_velocity_packet(sample['wx'], sample['wy'], sample['wz'])
        time.sleep(0.001)  # Small delay
        
        # Send angle packet (0x53)
        self.send_angle_packet(sample['roll'], sample['pitch'], sample['yaw'])
    
    def send_acceleration_packet(self, ax, ay, az):
        """Send WT60 acceleration packet (0x55 0x51)"""
        ax_raw = int((ax / 9.81) / 16.0 * 32768.0)
        ay_raw = int((ay / 9.81) / 16.0 * 32768.0)
        az_raw = int((az / 9.81) / 16.0 * 32768.0)
        
        ax_raw = max(-32768, min(32767, ax_raw))
        ay_raw = max(-32768, min(32767, ay_raw))
        az_raw = max(-32768, min(32767, az_raw))
        
        data = struct.pack('<3h', ax_raw, ay_raw, az_raw)
        temp = struct.pack('<h', 0)
        
        packet = bytearray([0x55, 0x51])
        packet.extend(data)
        packet.extend(temp)
        
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        
        self.serial_port.write(packet)
    
    def send_angular_velocity_packet(self, wx, wy, wz):
        """Send WT60 angular velocity packet (0x55 0x52)"""
        wx_deg = math.degrees(wx)
        wy_deg = math.degrees(wy)
        wz_deg = math.degrees(wz)
        
        wx_raw = int(wx_deg / 2000.0 * 32768.0)
        wy_raw = int(wy_deg / 2000.0 * 32768.0)
        wz_raw = int(wz_deg / 2000.0 * 32768.0)
        
        wx_raw = max(-32768, min(32767, wx_raw))
        wy_raw = max(-32768, min(32767, wy_raw))
        wz_raw = max(-32768, min(32767, wz_raw))
        
        data = struct.pack('<3h', wx_raw, wy_raw, wz_raw)
        temp = struct.pack('<h', 0)
        
        packet = bytearray([0x55, 0x52])
        packet.extend(data)
        packet.extend(temp)
        
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        
        self.serial_port.write(packet)
    
    def send_angle_packet(self, roll, pitch, yaw):
        """Send WT60 angle packet (0x55 0x53)"""
        roll_raw = int(roll / math.pi * 32768.0)
        pitch_raw = int(pitch / math.pi * 32768.0)
        yaw_raw = int(yaw / math.pi * 32768.0)
        
        roll_raw = max(-32768, min(32767, roll_raw))
        pitch_raw = max(-32768, min(32767, pitch_raw))
        yaw_raw = max(-32768, min(32767, yaw_raw))
        
        data = struct.pack('<3h', roll_raw, pitch_raw, yaw_raw)
        temp = struct.pack('<h', 0)
        
        packet = bytearray([0x55, 0x53])
        packet.extend(data)
        packet.extend(temp)
        
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        
        self.serial_port.write(packet)
    
    def destroy_node(self):
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FileIMUSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()