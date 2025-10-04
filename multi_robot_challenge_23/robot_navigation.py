import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan # Importert for type hint

class RobotNavigation:
    def __init__(self, node_ref: Node, sensors):
        self.node = node_ref
        self.sensors = sensors
        self.node.get_logger().info('RobotNavigation klar.')

        # intern tilstand for enkel kollisjonsunngåelse
        self.is_turning = False
        self.turn_time_start = None
        self.TURN_SPEED = 0.5
        self.FORWARD_SPEED = 0.5
        self.THRESHOLD = 0.8

        # publiser 10 Hz
        self.timer = self.node.create_timer(0.1, self.timer_callback)

    # Hent avstand ved vinkel (grader) fra siste LIDAR
    def _range_at_deg(self, scan: LaserScan, deg, default=100.0):
        if scan is None or not scan.ranges:
            return default
        
        # Matematikk for å finne indeksen i ranges-listen
        rad = math.radians(deg)
        idx = int(round((rad - scan.angle_min) / scan.angle_increment))
        
        if idx < 0 or idx >= len(scan.ranges):
            return default
        
        d = scan.ranges[idx]
        if d is None or np.isnan(d) or np.isinf(d) or d == 0.0:
            return default
            
        return float(d)

    def timer_callback(self):
        scan = self.sensors.get_latest_scan()

        # Viser en advarsel og stopper inntil LIDAR data mottas
        if scan is None or not scan.ranges:
            self.node.get_logger().warn('Venter på gyldig LIDAR data...')
            self.node.publish_twist(0.0, 0.0)
            return

        # Venstre/høyre "foran" (~±12°).
        dL = self._range_at_deg(scan, +12.0)
        dR = self._range_at_deg(scan, -12.0)

        vel = Twist()

        if self.is_turning:
            # Fortsett å svinge i 2.0 sekunder
            elapsed = (self.node.get_clock().now() - self.turn_time_start).nanoseconds / 1e9
            if elapsed < 2.0:
                vel.linear.x = 0.0
                vel.angular.z = self.TURN_SPEED # Fortsetter å svinge venstre (positiv)
            else:
                # Ferdig med sving, bytt til kjøring fremover
                self.is_turning = False
                self.turn_time_start = None
                vel.linear.x = self.FORWARD_SPEED
                vel.angular.z = 0.0
        else:
            # Logikk for kollisjonsunngåelse
            if dL > self.THRESHOLD and dR > self.THRESHOLD:
                # Fri vei: Kjør rett frem
                vel.linear.x = self.FORWARD_SPEED
                vel.angular.z = 0.0
            elif dL <= self.THRESHOLD and dR > self.THRESHOLD:
                # Hinder på venstre side: Sving høyre
                vel.linear.x = 0.0
                vel.angular.z = -self.TURN_SPEED
            elif dR <= self.THRESHOLD and dL > self.THRESHOLD:
                # Hinder på høyre side: Sving venstre
                vel.linear.x = 0.0
                vel.angular.z = self.TURN_SPEED
            else:
                # Hinder foran/på begge sider: Start 2 sekunders sving
                self.is_turning = True
                self.turn_time_start = self.node.get_clock().now()
                vel.linear.x = 0.0
                vel.angular.z = self.TURN_SPEED # Start sving venstre

        # Publiser via helper i MyRobotNode
        self.node.publish_twist(vel.linear.x, vel.angular.z)