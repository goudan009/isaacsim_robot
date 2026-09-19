#!/usr/bin/env python3
"""
IMU Publisher Node - Publishes IMU data from Isaac Sim to ROS2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header


class ImuPublisher(Node):
    """Publisher for IMU data from Isaac Sim."""

    def __init__(self):
        super().__init__('imu_publisher')

        self.get_logger().info('Initializing IMU Publisher Node')

        # Parameters
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 100.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Publisher
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_imu)

        self.get_logger().info(f'IMU publisher initialized')
        self.get_logger().info(f'Publishing at {self.publish_rate} Hz')

    def publish_imu(self):
        """Publish IMU data."""
        timestamp = self.get_clock().now().to_msg()

        # Create IMU message
        # TODO: Get actual IMU data from Isaac Sim
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = timestamp
        imu_msg.header.frame_id = self.frame_id

        # Placeholder: Zero values
        imu_msg.orientation.w = 1.0
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0

        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0

        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81

        # Covariance matrices (diagonal, moderate uncertainty)
        imu_msg.orientation_covariance = [0.01] * 9
        imu_msg.angular_velocity_covariance = [0.001] * 9
        imu_msg.linear_acceleration_covariance = [0.01] * 9

        self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
