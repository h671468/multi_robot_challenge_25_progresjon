#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Bug2Navigator:
    """
    BUG2 navigasjonsalgoritme - EN ansvar: BUG2 navigasjon
    
    Single Responsibility: Kun BUG2 algoritme implementasjon
    """
    
    # --- INNSTILLINGER ---
    FORWARD_SPEED = 0.3
    TURN_SPEED = 0.5
    THRESHOLD = 0.8
    GOAL_THRESHOLD = 0.5
    P_GAIN = 1.0
    
    # BUG2 States
    GO_TO_GOAL = "GO_TO_GOAL"
    WALL_FOLLOWING = "WALL_FOLLOWING"
    GOAL_REACHED = "GOAL_REACHED"
    
    def __init__(self, node_ref: Node):
        self.node = node_ref
        
        # BUG2 state
        self.state = self.GO_TO_GOAL
        self.target_position = None
        self.robot_position = (0.0, 0.0)
        self.robot_orientation = 0.0
        
        # BUG2 specific variables
        self.hit_point = None  # Posisjon hvor roboten traff hindringen
        self.leave_point = None  # Posisjon hvor roboten forlot hindringen
        self.m_line = None  # Linje fra start til mål
        self.start_position = None
        self.wall_following_direction = 1  # 1 for høyre, -1 for venstre
        
        # Wall following state
        self.is_turning = False
        self.turn_time_start = None
        
        # Setup publisher
        self.cmd_vel_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        
        self.node.get_logger().info('🐛 Bug2Navigator initialisert')

    def set_goal(self, position: tuple):
        """Sett nytt mål for BUG2 navigasjon"""
        self.target_position = position
        self.state = self.GO_TO_GOAL
        self.hit_point = None
        self.leave_point = None
        self.m_line = None
        self.start_position = self.robot_position
        self.node.get_logger().info(f'🐛 BUG2: Nytt mål satt: {position}')

    def clear_goal(self):
        """Fjern aktivt mål"""
        self.target_position = None
        self.state = self.GO_TO_GOAL
        self.node.get_logger().info('🐛 BUG2: Mål fjernet')

    def update_robot_pose(self, position: tuple, orientation: float):
        """Oppdater robot posisjon og orientering"""
        self.robot_position = position
        self.robot_orientation = orientation
        
        # Sett start posisjon hvis ikke satt
        if self.start_position is None:
            self.start_position = position

    def navigate(self, msg: LaserScan) -> bool:
        """
        BUG2 navigasjon hovedfunksjon
        
        Returns:
            bool: True hvis mål er nådd
        """
        if not self.target_position:
            return False

        # Sjekk om mål er nådd
        if self.is_goal_reached():
            self.node.get_logger().info('🐛 BUG2: GOAL REACHED!')
            self.clear_goal()
            return True

        # BUG2 state machine
        if self.state == self.GO_TO_GOAL:
            self.go_to_goal_state(msg)
        elif self.state == self.WALL_FOLLOWING:
            self.wall_following_state(msg)
        
        return False

    def go_to_goal_state(self, msg: LaserScan):
        """GO_TO_GOAL state - naviger direkte mot mål"""
        # Hent sensor data
        dL = self._range_at_deg(msg, +15.0)
        dR = self._range_at_deg(msg, -15.0)
        dF = self._range_at_deg(msg, 0.0)
        
        # Sjekk for hindringer
        if dF < self.THRESHOLD:
            # Hindring detektert - bytt til wall following
            self.hit_point = self.robot_position
            self.state = self.WALL_FOLLOWING
            self.is_turning = True
            self.turn_time_start = self.node.get_clock().now()
            self.node.get_logger().info(f'🐛 BUG2: Hindring detektert ved {self.hit_point} - bytter til wall following')
            return
        
        # Naviger mot mål
        self.navigate_towards_goal()

    def wall_following_state(self, msg: LaserScan):
        """WALL_FOLLOWING state - følg vegg til m-line"""
        # Hent sensor data
        dL = self._range_at_deg(msg, +15.0)
        dR = self._range_at_deg(msg, -15.0)
        dF = self._range_at_deg(msg, 0.0)
        
        # Sjekk om vi er tilbake på m-line og nærmere målet
        if self.is_on_m_line() and self.is_closer_to_goal():
            self.leave_point = self.robot_position
            self.state = self.GO_TO_GOAL
            self.is_turning = False
            self.node.get_logger().info(f'🐛 BUG2: Tilbake på m-line ved {self.leave_point} - bytter til go-to-goal')
            return
        
        # Fortsett wall following
        self.follow_wall(msg)

    def navigate_towards_goal(self):
        """Naviger direkte mot mål"""
        if not self.target_position:
            return
        
        # Beregn retning til mål
        dx = self.target_position[0] - self.robot_position[0]
        dy = self.target_position[1] - self.robot_position[1]
        distance_to_goal = math.sqrt(dx*dx + dy*dy)
        
        # Beregn ønsket heading
        desired_heading = math.atan2(dy, dx)
        heading_error = desired_heading - self.robot_orientation
        
        # Normaliser vinkel
        heading_error = self.normalize_angle(heading_error)
        
        # P-kontroll for heading
        angular_z = self.P_GAIN * heading_error
        
        # Adaptiv lineær hastighet
        linear_x = self.calculate_adaptive_speed(distance_to_goal)
        
        self.publish_twist(linear_x, angular_z)

    def follow_wall(self, msg: LaserScan):
        """Wall following logikk"""
        dL = self._range_at_deg(msg, +15.0)
        dR = self._range_at_deg(msg, -15.0)
        dF = self._range_at_deg(msg, 0.0)
        
        # Beregn elapsed time for turning
        elapsed = 0.0
        if self.is_turning and self.turn_time_start:
            elapsed = (self.node.get_clock().now() - self.turn_time_start).nanoseconds / 1e9
        
        linear_x = 0.0
        angular_z = 0.0
        
        if self.is_turning:
            # Fortsett å svinge
            if elapsed < 2.0:
                linear_x = 0.0
                angular_z = self.TURN_SPEED
            else:
                self.is_turning = False
                self.turn_time_start = None
        else:
            # Wall following logikk
            if dF < self.THRESHOLD:
                # Hindring foran - sving
                self.is_turning = True
                self.turn_time_start = self.node.get_clock().now()
                linear_x = 0.0
                angular_z = self.TURN_SPEED
            elif dL > self.THRESHOLD and dR > self.THRESHOLD:
                # Fri vei - gå fremover
                linear_x = self.FORWARD_SPEED
                angular_z = 0.0
            elif dL <= self.THRESHOLD and dR > self.THRESHOLD:
                # Vegg til venstre - sving høyre
                linear_x = self.FORWARD_SPEED * 0.5
                angular_z = -0.3
            elif dR <= self.THRESHOLD and dL > self.THRESHOLD:
                # Vegg til høyre - sving venstre
                linear_x = self.FORWARD_SPEED * 0.5
                angular_z = 0.3
            else:
                # Begge sider har hindringer - sving
                linear_x = 0.0
                angular_z = self.TURN_SPEED
        
        self.publish_twist(linear_x, angular_z)

    def is_goal_reached(self) -> bool:
        """Sjekk om mål er nådd"""
        if not self.target_position:
            return False
        
        distance = math.sqrt(
            (self.target_position[0] - self.robot_position[0])**2 +
            (self.target_position[1] - self.robot_position[1])**2
        )
        
        return distance <= self.GOAL_THRESHOLD

    def is_on_m_line(self) -> bool:
        """Sjekk om roboten er på m-line (linje fra start til mål)"""
        if not self.target_position or not self.start_position:
            return False
        
        # Beregn avstand fra robot til m-line
        distance_to_m_line = self.distance_to_line(
            self.start_position, self.target_position, self.robot_position
        )
        
        # Toleranse for å være "på" m-line
        tolerance = 0.3
        return distance_to_m_line <= tolerance

    def is_closer_to_goal(self) -> bool:
        """Sjekk om roboten er nærmere målet enn hit_point"""
        if not self.hit_point or not self.target_position:
            return False
        
        distance_from_hit = math.sqrt(
            (self.target_position[0] - self.hit_point[0])**2 +
            (self.target_position[1] - self.hit_point[1])**2
        )
        
        distance_from_current = math.sqrt(
            (self.target_position[0] - self.robot_position[0])**2 +
            (self.target_position[1] - self.robot_position[1])**2
        )
        
        return distance_from_current < distance_from_hit

    def distance_to_line(self, line_start: tuple, line_end: tuple, point: tuple) -> float:
        """Beregn avstand fra punkt til linje"""
        x0, y0 = point
        x1, y1 = line_start
        x2, y2 = line_end
        
        # Avstand fra punkt til linje formel
        numerator = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
        denominator = math.sqrt((y2 - y1)**2 + (x2 - x1)**2)
        
        if denominator == 0:
            return 0
        
        return numerator / denominator

    def calculate_adaptive_speed(self, distance_to_goal: float) -> float:
        """Beregn adaptiv hastighet basert på avstand til mål"""
        if distance_to_goal > 2.0:
            return self.FORWARD_SPEED
        elif distance_to_goal > 1.0:
            return self.FORWARD_SPEED * 0.7
        else:
            return self.FORWARD_SPEED * 0.4

    def normalize_angle(self, angle: float) -> float:
        """Normaliser vinkel til [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _range_at_deg(self, scan: LaserScan, deg: float, default: float = 100.0) -> float:
        """Hent avstand ved vinkel (grader) fra LIDAR"""
        if scan is None or not scan.ranges:
            return default
        
        rad = math.radians(deg)
        idx = int(round((rad - scan.angle_min) / scan.angle_increment))
        
        if idx < 0 or idx >= len(scan.ranges):
            return default
        
        d = scan.ranges[idx]
        if d is None or math.isnan(d) or math.isinf(d) or d == 0.0:
            return default
            
        return float(d)

    def publish_twist(self, linear_x: float, angular_z: float):
        """Publiserer bevegelseskommandoer"""
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist_msg)

    def stop_robot(self):
        """Stopper robot bevegelse"""
        self.publish_twist(0.0, 0.0)
