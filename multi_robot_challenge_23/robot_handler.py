#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class RobotHandler(Node):
    def __init__(self):
        super().__init__('robot_handler')
        self.get_logger().info('Robot handler node started')

def main(args=None):
    rclpy.init(args=args)
    robot_handler = RobotHandler()
    try:
        rclpy.spin(robot_handler)
    except KeyboardInterrupt:
        pass
    finally:
        robot_handler.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
