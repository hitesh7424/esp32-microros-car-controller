#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, MultiArrayDimension

class JoystickToWheelsNode(Node):
    def __init__(self):
        super().__init__('joystick_to_wheels_node')
        self.declare_parameter('max_speed', 100)
        self.subscription = self.create_subscription(
            Twist,
            '/controller/joystick_cmd',
            self.joystick_callback,
            10)
        self.publisher = self.create_publisher(Int32MultiArray, '/cmd_vel', 10)
        self.get_logger().info('Joystick to Wheels node started')

    def joystick_callback(self, msg):
        # Map Twist fields directly to wheel speeds
        def clamp(val):
            return int(max(-200, min(200, val)))
        s1 = clamp(msg.linear.x)   # Front Left
        s2 = clamp(msg.linear.y)   # Front Right
        s3 = clamp(msg.angular.x)  # Rear Left
        s4 = clamp(msg.angular.y)  # Rear Right
        speeds = [s1, s2, s3, s4]
        out_msg = Int32MultiArray()
        out_msg.layout.dim.append(MultiArrayDimension(label="wheels", size=4, stride=4))
        out_msg.layout.data_offset = 0
        out_msg.data = speeds
        self.publisher.publish(out_msg)
        self.get_logger().info(f'Published wheel speeds: {speeds}')

def main(args=None):
    rclpy.init(args=args)
    node = JoystickToWheelsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
