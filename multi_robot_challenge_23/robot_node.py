
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2 # Sikrer at OpenCV er lastet

from .robot_navigation import RobotNavigation
from .sensor_handler import SensorHandler

class MyRobotNode(Node):
    
    def __init__(self):
        super().__init__('robot_node')

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.sensors = SensorHandler(self)

        # Dette starter navigasjonslogikken
        self.navigation = RobotNavigation(self, self.sensors)

    def publish_twist(self, linear_x, angular_z):
        """Hjelpefunksjon for å sende hastighetskommandoer."""
        msg = Twist()
        msg.linear.x = float(linear_x) if linear_x is not None else 0.0
        msg.angular.z = float(angular_z) if angular_z is not None else 0.0
        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = MyRobotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stopper roboten ved avslutning
        node.publish_twist(0.0, 0.0) 
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()