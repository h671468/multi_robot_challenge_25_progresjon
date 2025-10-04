import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError

class SensorHandler:
    
    def __init__(self, node: Node):
        self.node = node
        self.bridge = CvBridge()

        self._latest_scan = None
        self._latest_odom = None
        self._latest_image = None

        # Setter opp subscribers. Bruker relative topic-navn ('scan', 'odom')
        self.node.create_subscription(LaserScan, 'scan', self._scan_cb, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, 'odom', self._odom_cb, 10)
        self.node.create_subscription(Image, 'camera/image_raw', self._image_cb, qos_profile_sensor_data)

        self.node.get_logger().info('SensorHandler initialisert.')

    # --- Callbacks ---
    
    def _scan_cb(self, msg: LaserScan):
        self._latest_scan = msg

    def _odom_cb(self, msg: Odometry):
        self._latest_odom = msg

    def _image_cb(self, msg: Image):
        try:
            # Konverterer til BGR8 for OpenCV-kompatibilitet
            self._latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.node.get_logger().error(f"CvBridge Error: {e}")

    # --- Getters ---
    
    def get_latest_scan(self):
        return self._latest_scan

    def get_latest_odom(self):
        return self._latest_odom

    def get_latest_image(self):
        return self._latest_image
