#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class IMUVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')
        
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10)
        
        self.count = 0
        self.get_logger().info('IMU Visualizer started - subscribing to /imu/data')
        
    def imu_callback(self, msg):
        # Only print every 10th message
        self.count += 1
        if self.count % 10 != 0:
            return
            
        # Convert quaternion to Euler angles
        q = msg.orientation
        
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        wx = math.degrees(msg.angular_velocity.x)
        wy = math.degrees(msg.angular_velocity.y)
        wz = math.degrees(msg.angular_velocity.z)
        
        self.get_logger().info(
            f'\n'
            f'Orientation (deg): Roll={roll_deg:7.2f}  Pitch={pitch_deg:7.2f}  Yaw={yaw_deg:7.2f}\n'
            f'Ang Velocity (deg/s): X={wx:7.2f}  Y={wy:7.2f}  Z={wz:7.2f}\n'
            f'Lin Accel (m/s²): X={msg.linear_acceleration.x:7.2f}  Y={msg.linear_acceleration.y:7.2f}  Z={msg.linear_acceleration.z:7.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = IMUVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()