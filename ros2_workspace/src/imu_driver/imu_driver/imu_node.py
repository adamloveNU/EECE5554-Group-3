#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import serial
import struct
import math

class WT60IMUNode(Node):

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q
    
    def destroy_node(self):
        """Clean up on shutdown"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

    def __init__(self):
        super().__init__('imu_node')
        
        # Declare parameters
        self.declare_parameter('port', '/tmp/vserial2')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'imu_link')
        
        # Get parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Initialize serial connection
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f'Connected to WT60 on {port} at {baudrate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            return
        
        # Create publisher
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        # Storage for IMU data - track which packets we've received
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = self.frame_id
        
        # Flags to track received packets in current cycle
        self.has_accel = False
        self.has_gyro = False
        self.has_angle = False
        
        # Set covariance matrices
        self.imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        self.imu_msg.angular_velocity_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
        self.imu_msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        
        # Create timer to read data
        self.timer = self.create_timer(0.001, self.read_imu_data)  # 1ms polling
        
        self.packet_count = 0
        self.error_count = 0
        
    def read_imu_data(self):
        try:
            while self.serial_port.in_waiting >= 11:
                # Look for header byte
                header = self.serial_port.read(1)
                if len(header) == 0:
                    continue
                    
                if header[0] != 0x55:
                    continue
                
                # Read packet type
                packet_type_byte = self.serial_port.read(1)
                if len(packet_type_byte) == 0:
                    continue
                packet_type = packet_type_byte[0]
                
                # Read data (6 bytes for xyz + 2 bytes temp)
                data = self.serial_port.read(8)
                if len(data) < 8:
                    continue
                
                # Read checksum
                checksum_byte = self.serial_port.read(1)
                if len(checksum_byte) == 0:
                    continue
                checksum = checksum_byte[0]
                
                # Verify checksum
                calc_checksum = (0x55 + packet_type + sum(data)) & 0xFF
                if calc_checksum != checksum:
                    self.error_count += 1
                    if self.error_count % 100 == 1:
                        self.get_logger().warn(f'Checksum mismatch (errors: {self.error_count})')
                    continue
                
                # Parse based on packet type
                if packet_type == 0x51:  # Acceleration
                    ax_raw, ay_raw, az_raw = struct.unpack('<3h', data[0:6])
                    
                    ax = (ax_raw / 32768.0) * 16.0 * 9.81
                    ay = (ay_raw / 32768.0) * 16.0 * 9.81
                    az = (az_raw / 32768.0) * 16.0 * 9.81
                    
                    self.imu_msg.linear_acceleration.x = ax
                    self.imu_msg.linear_acceleration.y = ay
                    self.imu_msg.linear_acceleration.z = az
                    self.has_accel = True
                    
                elif packet_type == 0x52:  # Angular velocity
                    wx_raw, wy_raw, wz_raw = struct.unpack('<3h', data[0:6])
                    
                    wx = (wx_raw / 32768.0) * 2000.0 * math.pi / 180.0
                    wy = (wy_raw / 32768.0) * 2000.0 * math.pi / 180.0
                    wz = (wz_raw / 32768.0) * 2000.0 * math.pi / 180.0
                    
                    self.imu_msg.angular_velocity.x = wx
                    self.imu_msg.angular_velocity.y = wy
                    self.imu_msg.angular_velocity.z = wz
                    self.has_gyro = True
                    
                elif packet_type == 0x53:  # Angle
                    roll_raw, pitch_raw, yaw_raw = struct.unpack('<3h', data[0:6])
                    
                    roll = (roll_raw / 32768.0) * math.pi
                    pitch = (pitch_raw / 32768.0) * math.pi
                    yaw = (yaw_raw / 32768.0) * math.pi
                    
                    self.imu_msg.orientation = self.euler_to_quaternion(roll, pitch, yaw)
                    self.has_angle = True
                
                # Only publish when we have all three measurements
                if self.has_accel and self.has_gyro and self.has_angle:
                    # Update timestamp
                    self.imu_msg.header.stamp = self.get_clock().now().to_msg()
                    
                    # Publish complete message
                    self.imu_pub.publish(self.imu_msg)
                    self.packet_count += 1
                    
                    # Reset flags for next cycle
                    self.has_accel = False
                    self.has_gyro = False
                    self.has_angle = False
                    
                    if self.packet_count % 100 == 0:
                        self.get_logger().info(
                            f'Published {self.packet_count} messages (errors: {self.error_count})'
                        )
                    
        except Exception as e:
            self.get_logger().error(f'Error reading IMU: {e}')
def main(args=None):
    rclpy.init(args=args)
    node = WT60IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()