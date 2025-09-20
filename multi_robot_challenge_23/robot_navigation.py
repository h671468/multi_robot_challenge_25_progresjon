import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RobotNavigation:
    def __init__(self, node_ref: Node):
        self.node = node_ref
        self.node.get_logger().info('Robot_Navigation-klassen er klar.')

        # Ikke opprett publisher og subscriber her, de blir sendt fra MyRobotNode
        # Variabler for å lagre LIDAR-data
        self.lidar_left_front = 100
        self.lidar_right_front = 100

        #variabler for navigasjon
        self.is_turning = False
        self.turn_time_start = None
        
        # Oppretter en timer for å publisere bevegelseskommandoer
        timer_period = 0.1
        self.timer = self.node.create_timer(timer_period, self.timer_callback)

    def process_scan(self, msg):
        self.lidar_left_front = msg.ranges[12]
        self.lidar_right_front = msg.ranges[348]

    def process_odom(self, msg):
        self.node.get_logger().info('Mottok odometri-data')

    def timer_callback(self):
        vel_msg = Twist()
        
        threshold = 0.8
        forward_speed = 0.5
        turn_speed = 0.5
        dL = self.lidar_left_front
        dR = self.lidar_right_front

        # Robotens bevegelseslogikk
        if self.is_turning:
            if (self.node.get_clock().now() - self.turn_time_start).nanoseconds /1e9 < 2.0:
                #forsetter å svinge i 2 sekunder
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = turn_speed
            else:
                #svingen er ferdig, tilbake til normal navigasjon
                self.is_turning = False
                self.turn_time_start = None
                vel_msg.linear.x = forward_speed
                vel_msg.angular.z = 0.0
        else:
            # Normal navigasjonslogikk
            if dL > threshold and dR > threshold:
                # Ingen hindring foran, kjør fremover
                vel_msg.linear.x = forward_speed
                vel_msg.angular.z = 0.0
            elif dL <= threshold and dR > threshold:
                # Hindring til venstre, sving til høyre
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = -turn_speed
            elif dR <= threshold and dL > threshold:
                # Hindring til høyre, sving til venstre
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = turn_speed
            else: # dL <= threshold and dR <= threshold
                # Hindring både til venstre og høyre, start svingmanøver
                self.is_turning = True
                self.turn_time_start = self.node.get_clock().now()
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = turn_speed

        # Bruk publish_twist-metoden i MyRobotNode
        self.node.publish_twist(vel_msg.linear.x, vel_msg.angular.z)