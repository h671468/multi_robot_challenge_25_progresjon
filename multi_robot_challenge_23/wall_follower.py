#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollower:
    """
    Wall Following klasse - EN ansvar: Wall following navigasjon
    
    Single Responsibility: Kun wall following logikk
    """
    
    # --- INNSTILLINGER ---
    FORWARD_SPEED = 0.3       # Standard hastighet fremover
    TURN_SPEED = 0.5          # Standard rotasjonshastighet
    THRESHOLD = 0.8           # Avstand for kollisjonsunngåelse
    WALL_DISTANCE = 0.5       # Ønsket avstand fra vegg
    
    def __init__(self, node_ref: Node):
        self.node = node_ref
        
        # Wall following state
        self.is_turning = False
        self.turn_time_start = None
        
        # Setup publisher
        from geometry_msgs.msg import Twist
        self.cmd_vel_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        
        self.node.get_logger().info('🧱 WallFollower initialisert')

    def follow_wall(self, msg: LaserScan, target_direction=None):
        """
        Wall following logikk
        
        Args:
            msg: LaserScan data
            target_direction: (x, y) retning for guided wall following
        """
        if not msg.ranges:
            self.stop_robot()
            return

        if target_direction:
            self.guided_wall_following(msg, target_direction)
        else:
            self.simple_wall_following(msg)

    def guided_wall_following(self, msg: LaserScan, target):
        """Wall following med retning mot mål"""
        dL = self._range_at_deg(msg, +15.0)
        dR = self._range_at_deg(msg, -15.0)
        dF = self._range_at_deg(msg, 0.0)

        # Kollisjonsunngåelse først
        if dF < self.THRESHOLD:
            self.is_turning = True
            self.turn_time_start = self.node.get_clock().now()
            self.publish_twist(0.0, self.TURN_SPEED)
        elif self.is_turning:
            # Fortsett sving
            elapsed = (self.node.get_clock().now() - self.turn_time_start).nanoseconds / 1e9
            if elapsed < 2.0:
                self.publish_twist(0.0, self.TURN_SPEED)
            else:
                self.is_turning = False
                self.turn_time_start = None
        else:
            # Wall following med retning mot mål
            if dL > self.THRESHOLD and dR > self.THRESHOLD:
                # Fri vei - naviger mot mål
                desired_heading = math.atan2(target[1], target[0])
                angular_z = 0.5 * desired_heading
                linear_x = self.FORWARD_SPEED
                self.publish_twist(linear_x, angular_z)
            else:
                # Standard wall following
                self.simple_wall_following(msg)

    def simple_wall_following(self, msg: LaserScan):
        """Enkel wall following logikk"""
        dL = self._range_at_deg(msg, +15.0)
        dR = self._range_at_deg(msg, -15.0)
        dF = self._range_at_deg(msg, 0.0)

        # Beregn elapsed time for turning
        elapsed = 0.0
        if self.is_turning and self.turn_time_start:
            elapsed = (self.node.get_clock().now() - self.turn_time_start).nanoseconds / 1e9

        # Enkel wall following logikk
        linear_x = 0.0
        angular_z = 0.0

        if self.is_turning:
            # Fortsett å svinge
            linear_x = 0.0
            angular_z = self.TURN_SPEED
        else:
            # Enkel kollisjonsunngåelse og wall following
            if dF < self.THRESHOLD:
                self.is_turning = True
                self.turn_time_start = self.node.get_clock().now()
                linear_x = 0.0
                angular_z = self.TURN_SPEED
            elif dL > self.THRESHOLD and dR > self.THRESHOLD:
                linear_x = self.FORWARD_SPEED
                angular_z = 0.0
            elif dL <= self.THRESHOLD and dR > self.THRESHOLD:
                linear_x = self.FORWARD_SPEED * 0.5
                angular_z = -0.3
            elif dR <= self.THRESHOLD and dL > self.THRESHOLD:
                linear_x = self.FORWARD_SPEED * 0.5
                angular_z = 0.3
            else:
                linear_x = 0.0
                angular_z = self.TURN_SPEED

        # Håndter turning state
        if dF < self.THRESHOLD and not self.is_turning:
            self.is_turning = True
            self.turn_time_start = self.node.get_clock().now()
        elif self.is_turning and elapsed >= 2.0:
            self.is_turning = False
            self.turn_time_start = None

        self.publish_twist(linear_x, angular_z)

    def _range_at_deg(self, scan: LaserScan, deg, default=100.0):
        """Hent avstand ved vinkel (grader) fra siste LIDAR"""
        if scan is None or not scan.ranges:
            return default
        
        # Matematikk for å finne indeksen i ranges-listen
        rad = math.radians(deg)
        idx = int(round((rad - scan.angle_min) / scan.angle_increment))
        
        if idx < 0 or idx >= len(scan.ranges):
            return default
        
        d = scan.ranges[idx]
        if d is None or math.isnan(d) or math.isinf(d) or d == 0.0:
            return default
            
        return float(d)

    def publish_twist(self, linear_x, angular_z):
        """Publiserer bevegelseskommandoer"""
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.node.cmd_vel_publisher.publish(twist_msg)

    def stop_robot(self):
        """Stopper robot bevegelse"""
        self.publish_twist(0.0, 0.0)