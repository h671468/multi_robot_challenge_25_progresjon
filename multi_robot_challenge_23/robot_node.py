import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

#Importerer klassene for logikken 
#from .robot_communication import RobotCommunication
#from .robot_controller import RobotController
from .robot_navigation import RobotNavigation 
from .sensor_handler import SensorHandler

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        # Opprett publisher og subscriber her
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
#        self.scan_subscriber = self.create_subscription(
#            LaserScan, 'scan', self.scan_callback, 10
#        )
#        self.odom_subscriber = self.create_subscription(
#            Odometry, 'odom', self.odom_callback, 10
#        )

        # Initialiser klassene, og send inn publisher- og subscriber-objektene
        self.navigation = RobotNavigation(self)
        #self.controller = RobotController(self)
        #self.communication = RobotCommunication(self)
        self.sensor = SensorHandler(self)

#    def scan_callback(self, msg):
        # Send LIDAR-data til navigasjonslogikken
#        self.navigation.process_scan(msg)

#    def odom_callback(self, msg):
        # Send odometri-data til navigasjonslogikken
#        self.navigation.process_odom(msg)

    def publish_twist(self, linear_x, angular_z):
        # Hjelpefunksjon for å publisere bevegelseskommandoer
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    robot_node = MyRobotNode()
    rclpy.spin(robot_node)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()