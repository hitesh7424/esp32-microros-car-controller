#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class WheelSpeedController(Node):
    def __init__(self):
        super().__init__("wheel_speed_controller")
        self.publisher_ = self.create_publisher(Twist, "/car1/wheel_speeds", 1)
        self.subscription = self.create_subscription(
            Twist, "/ctrl1/cmd_vel", self.cmd_vel_callback, 1
        )
        self.get_logger().info("Wheel Speed Controller Node has been started.")

    def cmd_vel_callback(self, msg):
        if msg.linear.x > 0 and msg.angular.z == 0:
            self.move_forward(msg.linear.x * 200)
        elif msg.linear.x < 0 and msg.angular.z == 0:
            self.move_reverse(msg.linear.x * 200)
        elif msg.angular.z > 0 and msg.linear.x == 0:
            self.turn_left(msg.angular.z * 150)
        elif msg.angular.z < 0 and msg.linear.x == 0:
            self.turn_right(msg.angular.z * 150)
        elif msg.linear.x == 0 and msg.angular.z == 0:
            self.stop()
        else:
            self.get_logger().warn("Unsupported movement command received.")

    def move_forward(self, speed):
        wheel_speeds = Twist()
        wheel_speeds.linear.x = -speed  # Front left wheel
        wheel_speeds.angular.x = -speed  # Front right wheel
        wheel_speeds.linear.y = -speed  # Rear left wheel
        wheel_speeds.angular.y = -speed  # Rear right wheel
        self.publish_wheel_speeds(wheel_speeds, "Moving forward")

    def move_reverse(self, speed):
        wheel_speeds = Twist()
        wheel_speeds.linear.x = -speed  # Front left wheel
        wheel_speeds.angular.x = -speed  # Front right wheel
        wheel_speeds.linear.y = -speed  # Rear left wheel
        wheel_speeds.angular.y = -speed  # Rear right wheel
        self.publish_wheel_speeds(wheel_speeds, "Moving reverse")

    def turn_left(self, speed):
        wheel_speeds = Twist()
        wheel_speeds.linear.x = -speed  # Front left wheel
        wheel_speeds.angular.x = speed  # Front right wheel
        wheel_speeds.linear.y = -speed  # Rear left wheel
        wheel_speeds.angular.y = speed  # Rear right wheel
        self.publish_wheel_speeds(wheel_speeds, "Turning left")

    def turn_right(self, speed):
        wheel_speeds = Twist()
        wheel_speeds.linear.x = -speed  # Front left wheel
        wheel_speeds.angular.x = speed  # Front right wheel
        wheel_speeds.linear.y = -speed  # Rear left wheel
        wheel_speeds.angular.y = speed  # Rear right wheel
        self.publish_wheel_speeds(wheel_speeds, "Turning right")

    def stop(self):
        wheel_speeds = Twist()
        wheel_speeds.linear.x = 0.0
        wheel_speeds.angular.x = 0.0
        wheel_speeds.linear.y = 0.0
        wheel_speeds.angular.y = 0.0
        self.publish_wheel_speeds(wheel_speeds, "Stopping")

    def publish_wheel_speeds(self, wheel_speeds, action):
        self.publisher_.publish(wheel_speeds)
        self.get_logger().info(
            f"{action}: Front -> linear.x={wheel_speeds.linear.x}, angular.x={wheel_speeds.angular.x}; Rear -> linear.y={wheel_speeds.linear.y}, angular.y={wheel_speeds.angular.y}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = WheelSpeedController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
