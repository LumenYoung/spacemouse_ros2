import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pyspacemouse
from pyspacemouse.pyspacemouse import SpaceNavigator, DeviceSpec


class SpaceMousePublisher(Node):
    def __init__(self):
        super().__init__('spacemouse_publisher')
        self.publisher_ = self.create_publisher(Twist, "spacemouse_cmd", 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.device = pyspacemouse.open()
        if not self.device:
            self.get_logger().error("Failed to open SpaceMouse device.")
            rclpy.shutdown()

    def timer_callback(self):
        state = pyspacemouse.read()
        if state:
            msg = Twist()
            # initially the state values would be integer
            msg.linear.x = float(state.x)
            msg.linear.y = float(state.y)
            msg.linear.z = float(state.z)
            msg.angular.x = float(state.roll)
            msg.angular.y = float(state.pitch)
            msg.angular.z = float(state.yaw)
            self.publisher_.publish(msg)
            # self.get_logger().info(f"Publishing: {msg}")


def main(args=None):
    rclpy.init(args=args)
    spacemouse_publisher = SpaceMousePublisher()
    try:
        rclpy.spin(spacemouse_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        spacemouse_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
