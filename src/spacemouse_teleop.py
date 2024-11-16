#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from franky import Affine, CartesianMotion, Robot, ReferenceType


class SpaceMouseTeleop(Node):
    def __init__(self):
        super().__init__('spacemouse_teleop')
        
        # Connect to robot
        self.robot = Robot("192.168.3.100")
        self.robot.relative_dynamics_factor = 0.05
        
        # Subscribe to spacemouse commands
        self.subscription = self.create_subscription(
            Twist,
            'spacemouse_cmd',
            self.cmd_callback,
            10)
        
        # Scale factors for converting spacemouse input to robot motion
        self.linear_scale = 0.001  # Scale down linear motion (m)
        self.angular_scale = 0.001  # Scale down angular motion (rad)
        
        self.get_logger().info("SpaceMouse teleop node started")

    def cmd_callback(self, msg: Twist):
        # Convert spacemouse command to relative motion
        linear_motion = [
            msg.linear.x * self.linear_scale,
            msg.linear.y * self.linear_scale, 
            msg.linear.z * self.linear_scale
        ]
        
        # Create relative motion command
        motion = CartesianMotion(
            Affine(linear_motion),
            ReferenceType.Relative
        )
        
        # Execute motion asynchronously
        try:
            self.robot.move(motion, asynchronous=True)
        except Exception as e:
            self.get_logger().error(f"Motion execution failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    teleop_node = SpaceMouseTeleop()
    
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure to stop any ongoing motion
        teleop_node.robot.join_motion()
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
