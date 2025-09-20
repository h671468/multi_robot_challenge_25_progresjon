import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class SensorHandler:
    def __init__(self, node: Node):
        self.node = node
        self.bridge = CvBridge()
        self.latest_scan = None
        self.latest_odom = None
        self.latest_image = None

        #Abonnenter for å motta sensordata
        self.node.scan_subscriber = self.node.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.node.odom_subscriber = self.node.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10
        )
        self.node.image_subscriber = self.node.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.node.get_logger().info('Sensor_Handler initialisert.')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def image_callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.node.get_logger().error(f"CvBridge Error : {e}")


    def get_latest_scan(self):
        return self.latest_scan
    
    def get_latest_odom(self):
        return self.latest_odom
    
    def get_latest_image(self):
        return self.latest_image