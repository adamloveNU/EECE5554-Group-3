#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import json
import cv2
import pickle
import os

class PiCameraNode(Node):
    def __init__(self):
        super().__init__('pi_camera_node')
        
        # Declare parameters
        self.declare_parameter('pipe_path', '/tmp/camera_pipe')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('camera_name', 'pi_camera')
        self.declare_parameter('frame_id', 'camera_link')
        
        # Get parameters
        self.pipe_path = self.get_parameter('pipe_path').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.camera_name = self.get_parameter('camera_name').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Check if pipe exists
        if not os.path.exists(self.pipe_path):
            self.get_logger().error(f'Camera pipe not found: {self.pipe_path}')
            self.get_logger().error('Make sure the camera simulator is running first!')
            return
        
        self.get_logger().info(f'Connected to camera pipe: {self.pipe_path}')
        
        # Create publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera setup
        with open(os.path.join(self.pipe_path, "meta.json"), 'r') as f:
            meta = json.load(f)
        width = meta['width']
        height = meta['height']
        self.camera_info = self.create_camera_info(width, height)
        self.get_logger().info(f'Camera initialized: {width}x{height}')
        
        # Frame counter
        #self.frame_count = 0
        
        # Create timer to read and publish frames
        timer_period = 1.0 / self.frame_rate
        self.get_logger().info(f'Camera started at {self.frame_rate} fps')
        self.last_mtime = 0
        self.timer = self.create_timer(timer_period, self.read_and_publish)
        
    def create_camera_info(self, width, height):
        """Create camera info message with Pi Camera V2 specs"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.frame_id
        
        # Image dimensions
        camera_info.height = height
        camera_info.width = width
        
        # Distortion model
        camera_info.distortion_model = "plumb_bob"
        
        # Distortion coefficients
        camera_info.d = [0.1, -0.25, 0.0, 0.0, 0.0]
        
        # Intrinsic camera matrix
        fx = 1000.0
        fy = 1000.0
        cx = width / 2.0
        cy = height / 2.0
        
        camera_info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]
        
        # Rectification matrix
        camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        
        # Projection matrix
        camera_info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        return camera_info
    
    def read_and_publish(self):
        #"""Read frame from pipe and publish"""
        #try:
        # check for a new video frame
        self.mtime = os.path.getmtime(os.path.join(self.pipe_path, "frame.npy"))
        if self.mtime == self.last_mtime:
            return

        # load new video frame
        frame = np.load(os.path.join(self.pipe_path, "frame.npy"))
        if frame is None:
            return
        
        # Create ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = self.frame_id
        
        # Update camera info timestamp
        self.camera_info.header.stamp = image_msg.header.stamp
        
        # Publish
        self.image_pub.publish(image_msg)
        self.camera_info_pub.publish(self.camera_info)
        
        #self.frame_count += 1
        
        # Log progress
        #if self.frame_count % 100 == 0:
        #    self.get_logger().info(f'Published {self.frame_count} frames')
            
        #except EOFError:
        #    # Pipe temporarily closed during loop - just skip this cycle
        #    pass
        #except FileNotFoundError:
        #    # Pipe doesn't exist - simulator not running
        #    pass
        #except Exception as e:
        #    # Other errors - only log occasionally
        #    if self.frame_count % 300 == 0:
        #        self.get_logger().debug(f'Read error: {type(e).__name__}')
        #    pass

def main(args=None):
    rclpy.init(args=None)
    node = PiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
