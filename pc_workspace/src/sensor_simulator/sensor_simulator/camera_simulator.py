#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import os
import pickle
import time

class CameraSimulator(Node):
    def __init__(self):
        super().__init__('camera_simulator')
        
        # Declare parameters
        self.declare_parameter('video_file', '')
        self.declare_parameter('pipe_path', '/tmp/camera_pipe')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('playback_speed', 1.0)
        
        # Get parameters
        video_file = self.get_parameter('video_file').value
        self.pipe_path = self.get_parameter('pipe_path').value
        self.fps = self.get_parameter('fps').value
        self.loop = self.get_parameter('loop').value
        self.playback_speed = self.get_parameter('playback_speed').value
        
        # Check if video file exists
        if not video_file:
            self.get_logger().error('No video file specified!')
            return
        
        if not os.path.exists(video_file):
            self.get_logger().error(f'Video file not found: {video_file}')
            return
        
        # Create named pipe if it doesn't exist
        if os.path.exists(self.pipe_path):
            os.remove(self.pipe_path)
        
        os.mkfifo(self.pipe_path)
        self.get_logger().info(f'Created camera pipe: {self.pipe_path}')
        
        # Open video file
        self.cap = cv2.VideoCapture(video_file)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open video file: {video_file}')
            return
        
        # Get video properties
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        
        self.get_logger().info(f'Loaded video: {video_file}')
        self.get_logger().info(f'Resolution: {self.width}x{self.height}')
        self.get_logger().info(f'Total frames: {self.total_frames}')
        self.get_logger().info(f'Target FPS: {self.fps}')
        
        # Frame counter
        self.frame_count = 0
        
        # Keep pipe file handle open
        self.pipe_file = None
        
        # Create timer to stream frames
        timer_period = (1.0 / self.fps) / self.playback_speed
        self.timer = self.create_timer(timer_period, self.stream_frame)
        
        self.get_logger().info('Waiting for camera driver to connect...')
        
    def stream_frame(self):
        """Read frame from video and write to pipe"""
        
        # Open pipe if not already open
        if self.pipe_file is None:
            try:
                self.pipe_file = open(self.pipe_path, 'wb', buffering=0)
                self.get_logger().info('Camera driver connected!')
            except Exception as e:
                # Pipe not ready yet
                return
        
        # Read frame
        ret, frame = self.cap.read()
        
        if not ret:
            if self.loop:
                self.get_logger().info('Looping video...')
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
                self.frame_count = 0
                
                if not ret:
                    self.get_logger().error('Failed to read video on loop')
                    return
            else:
                self.get_logger().info('Video playback complete.')
                self.timer.cancel()
                return
        
        try:
            # Serialize frame data
            frame_data = {
                'frame': frame,
                'width': self.width,
                'height': self.height,
                'timestamp': time.time()
            }
            
            # Write to pipe (keep pipe open)
            pickle.dump(frame_data, self.pipe_file)
            self.pipe_file.flush()  # Flush to ensure data is sent
                
            self.frame_count += 1
            
            if self.frame_count % 100 == 0:
                progress = (self.frame_count / self.total_frames) * 100
                self.get_logger().info(
                    f'Streamed {self.frame_count}/{self.total_frames} frames ({progress:.1f}%)'
                )
                
        except BrokenPipeError:
            # Driver disconnected
            self.get_logger().warn('Driver disconnected')
            if self.pipe_file:
                self.pipe_file.close()
                self.pipe_file = None
        except Exception as e:
            self.get_logger().error(f'Error streaming frame: {e}')
            if self.pipe_file:
                self.pipe_file.close()
                self.pipe_file = None
    
    def destroy_node(self):
        """Clean up"""
        if hasattr(self, 'pipe_file') and self.pipe_file:
            self.pipe_file.close()
        if hasattr(self, 'cap'):
            self.cap.release()
        if os.path.exists(self.pipe_path):
            os.remove(self.pipe_path)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()