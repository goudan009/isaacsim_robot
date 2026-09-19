#!/usr/bin/python3
"""Isaac-only endpoint adaptation for the upstream OpenArmX VR node."""

from __future__ import annotations

import rclpy
from openarmx_teleop_vr.openarmx_teleop_vr_node import OpenArmXTeleopVRNode


class OpenFlexIsaacVRArmNode(OpenArmXTeleopVRNode):
    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter("isaac_left_ee_frame", "openarmx_left_hand_tcp")
        self.declare_parameter("isaac_right_ee_frame", "openarmx_right_hand_tcp")
        left_frame = str(self.get_parameter("isaac_left_ee_frame").value)
        right_frame = str(self.get_parameter("isaac_right_ee_frame").value)
        solver = self.core.ik_solver
        for frame_name in (left_frame, right_frame):
            if not solver.model.existFrame(frame_name):
                raise ValueError(f"Isaac IK end-effector frame not found: {frame_name}")
        solver.left_ee_frame = left_frame
        solver.right_ee_frame = right_frame
        solver.left_ee_id = solver.model.getFrameId(left_frame)
        solver.right_ee_id = solver.model.getFrameId(right_frame)
        solver.reset()
        self.get_logger().info(
            f"Isaac IK endpoints: {left_frame}, {right_frame}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpenFlexIsaacVRArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
