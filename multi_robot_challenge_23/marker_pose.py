#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MarkerPose(Node):
    def __init__(self):
        super().__init__('marker_recognition')
        self.get_logger().info('Marker recognition node started')

def main(args=None):
    rclpy.init(args=args)
    marker_pose_node = MarkerPose()
    try:
        rclpy.spin(marker_pose_node)
    except KeyboardInterrupt:
        pass
    finally:
        marker_pose_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
