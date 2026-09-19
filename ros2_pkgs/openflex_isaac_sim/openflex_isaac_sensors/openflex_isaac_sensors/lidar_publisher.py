#!/usr/bin/env python3
"""
LiDAR Publisher Node - Publishes LiDAR point cloud from Isaac Sim to ROS2.

Supports Livox Mid360 LiDAR mounted on the head.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct


class LidarPublisher(Node):
    """Publisher for LiDAR data from Isaac Sim."""

    def __init__(self):
        super().__init__('lidar_publisher')

        self.get_logger().info('Initializing LiDAR Publisher Node')

        # Parameters
        self.declare_parameter('lidar_name', 'mid360')
        self.declare_parameter('frame_id', 'mid360_lidar_link')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('points_per_scan', 24000)

        self.lidar_name = self.get_parameter('lidar_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.points_per_scan = self.get_parameter('points_per_scan').value

        # Publisher
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, f'/lidar/{self.lidar_name}/points', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_pointcloud)

        self.get_logger().info(f'LiDAR publisher initialized: {self.lidar_name}')
        self.get_logger().info(f'Publishing at {self.publish_rate} Hz')

    def publish_pointcloud(self):
        """Publish point cloud data."""
        timestamp = self.get_clock().now().to_msg()

        # Create PointCloud2 message
        # TODO: Get actual point cloud data from Isaac Sim
        pc_msg = PointCloud2()
        pc_msg.header = Header()
        pc_msg.header.stamp = timestamp
        pc_msg.header.frame_id = self.frame_id

        # PointCloud2 fields
        pc_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        pc_msg.point_step = 16  # 4 floats * 4 bytes
        pc_msg.height = 1
        pc_msg.width = self.points_per_scan
        pc_msg.is_bigendian = False
        pc_msg.is_dense = True
        pc_msg.row_step = pc_msg.point_step * pc_msg.width

        # Placeholder: Generate dummy point cloud (empty for now)
        pc_msg.data = bytes(self.points_per_scan * pc_msg.point_step)

        self.pointcloud_pub.publish(pc_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
