#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Leader(Node):
    def __init__(self):
        super().__init__('leader')
        self.get_logger().info('Leader node started')

def main(args=None):
    rclpy.init(args=args)
    leader = Leader()
    try:
        rclpy.spin(leader)
    except KeyboardInterrupt:
        pass
    finally:
        leader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
