#!/usr/bin/env python3
"""
Isaac Sim Bridge Node - Main communication bridge between Isaac Sim and ROS2.

This node manages:
- Sensor data forwarding (Camera, LiDAR, IMU)
- Command forwarding (velocity, joint commands)
- Simulation control services (reset, pause, spawn)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class IsaacSimBridgeNode(Node):
    """Bridge node for Isaac Sim communication."""

    def __init__(self):
        super().__init__('isaac_sim_bridge')

        self.get_logger().info('Initializing Isaac Sim Bridge Node')

        # Parameters
        self.declare_parameter('isaac_sim_host', '127.0.0.1')
        self.declare_parameter('isaac_sim_port', 24102)
        self.declare_parameter('enable_sensors', True)
        self.declare_parameter('enable_control', True)

        self.isaac_host = self.get_parameter('isaac_sim_host').value
        self.isaac_port = self.get_parameter('isaac_sim_port').value
        self.enable_sensors = self.get_parameter('enable_sensors').value
        self.enable_control = self.get_parameter('enable_control').value

        # Publishers - forward Isaac Sim data to ROS2
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribers - forward ROS2 commands to Isaac Sim
        if self.enable_control:
            self.cmd_vel_sub = self.create_subscription(
                Twist, '/cmd_vel', self.cmd_vel_callback, 10)
            self.joint_cmd_sub = self.create_subscription(
                Float64MultiArray, '/joint_command', self.joint_cmd_callback, 10)

        # Services - simulation control
        self.reset_srv = self.create_service(Trigger, '/sim/reset', self.reset_callback)
        self.pause_srv = self.create_service(SetBool, '/sim/pause', self.pause_callback)

        # Status timer
        self.status_timer = self.create_timer(1.0, self.status_callback)

        self.get_logger().info(f'Bridge initialized: {self.isaac_host}:{self.isaac_port}')
        self.get_logger().info(f'Sensors: {self.enable_sensors}, Control: {self.enable_control}')

    def cmd_vel_callback(self, msg: Twist):
        """Forward velocity command to Isaac Sim."""
        # TODO: Send to Isaac Sim via socket/service
        pass

    def joint_cmd_callback(self, msg: Float64MultiArray):
        """Forward joint command to Isaac Sim."""
        # TODO: Send to Isaac Sim via socket/service
        pass

    def reset_callback(self, request, response):
        """Reset simulation."""
        self.get_logger().info('Resetting simulation')
        # TODO: Call Isaac Sim reset API
        response.success = True
        response.message = 'Simulation reset requested'
        return response

    def pause_callback(self, request, response):
        """Pause/unpause simulation."""
        action = 'Pausing' if request.data else 'Unpausing'
        self.get_logger().info(f'{action} simulation')
        # TODO: Call Isaac Sim pause API
        response.success = True
        response.message = f'Simulation {action.lower()}'
        return response

    def status_callback(self):
        """Periodic status check."""
        # TODO: Check Isaac Sim connection status
        pass


def main(args=None):
    rclpy.init(args=args)
    node = IsaacSimBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
