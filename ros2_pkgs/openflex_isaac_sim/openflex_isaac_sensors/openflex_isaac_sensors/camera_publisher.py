#!/usr/bin/env python3
"""
Camera Publisher Node - Publishes camera images from Isaac Sim to ROS2.

Supports multiple cameras:
- Head camera
- Chest camera
- Left/right wrist cameras
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import numpy as np


class CameraPublisher(Node):
    """Publisher for camera data from Isaac Sim."""

    def __init__(self):
        super().__init__('camera_publisher')

        self.get_logger().info('Initializing Camera Publisher Node')

        # Parameters
        self.declare_parameter('camera_name', 'head_camera')
        self.declare_parameter('frame_id', 'head_camera_optical_frame')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        self.camera_name = self.get_parameter('camera_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value

        # Publishers
        self.image_pub = self.create_publisher(
            Image, f'/camera/{self.camera_name}/image_raw', 10)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, f'/camera/{self.camera_name}/camera_info', 10)

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_camera_data)

        self.get_logger().info(f'Camera publisher initialized: {self.camera_name}')
        self.get_logger().info(f'Publishing at {self.publish_rate} Hz, Resolution: {self.width}x{self.height}')

    def publish_camera_data(self):
        """Publish camera image and info."""
        timestamp = self.get_clock().now().to_msg()

        # Create image message
        # TODO: Get actual image data from Isaac Sim
        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = timestamp
        img_msg.header.frame_id = self.frame_id
        img_msg.height = self.height
        img_msg.width = self.width
        img_msg.encoding = 'rgb8'
        img_msg.is_bigendian = 0
        img_msg.step = self.width * 3

        # Placeholder: Create dummy image data (black frame)
        img_msg.data = bytes(self.height * self.width * 3)

        self.image_pub.publish(img_msg)

        # Create camera info message
        cam_info = CameraInfo()
        cam_info.header = img_msg.header
        cam_info.height = self.height
        cam_info.width = self.width

        # Placeholder camera intrinsics (typical values)
        fx = fy = 500.0
        cx = self.width / 2.0
        cy = self.height / 2.0

        cam_info.k = [fx, 0.0, cx,
                      0.0, fy, cy,
                      0.0, 0.0, 1.0]
        cam_info.p = [fx, 0.0, cx, 0.0,
                      0.0, fy, cy, 0.0,
                      0.0, 0.0, 1.0, 0.0]
        cam_info.distortion_model = "plumb_bob"
        cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.camera_info_pub.publish(cam_info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
