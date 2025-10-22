#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info('Car controller node started')
        self.speed_pub = self.create_publisher(Int32MultiArray, 'wheel_speeds', 10)
        self.timer = self.create_timer(1.0, self.publish_speeds)

    def publish_speeds(self):
        # Example: set all wheels to speed 50
        speeds = [50, 50, 50, 50]  # [front_left, front_right, rear_left, rear_right]
        msg = Int32MultiArray()
        msg.data = [max(0, min(100, s)) for s in speeds]
        self.speed_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
