#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from .wall_follower import WallFollower
from .goal_navigator import GoalNavigator
from .big_fire_coordinator import BigFireCoordinator
from .aruco_detector import ArUcoDetector
from .robot_memory import RobotMemory

class SearchRescueCoordinator:
    """
    Search & Rescue koordinator - koordinerer alle komponenter
    
    Single Responsibility: Koordinering av komponenter
    """
    
    def __init__(self, node_ref: Node):
        self.node = node_ref
        self.robot_id = self.node.get_namespace().strip('/')
        
        # Robot position tracking
        self.robot_position = (0.0, 0.0)
        self.robot_orientation = 0.0
        
        # Initialize komponenter
        self.robot_memory = RobotMemory()
        self.wall_follower = WallFollower(node_ref)
        self.goal_navigator = GoalNavigator(node_ref)
        self.big_fire_coordinator = BigFireCoordinator(node_ref, self.robot_memory)
        self.aruco_detector = ArUcoDetector(node_ref, self.handle_aruco_detection)
        
        self.node.get_logger().info(f'🤖 SearchRescueCoordinator ({self.robot_id}) initialisert')

    def process_scan(self, msg: LaserScan):
        """Hovedfunksjon - koordinerer navigasjon"""
        if not msg.ranges:
            self.wall_follower.stop_robot()
            return

        # Big Fire koordinering har høyest prioritet
        big_fire_active = self.big_fire_coordinator.should_handle_big_fire()
        if big_fire_active:
            self.node.get_logger().info('🔥 BIG FIRE KOORDINERING AKTIV!')
            self.handle_big_fire(msg)
        else:
            # Standard wall following
            self.wall_follower.follow_wall(msg)

    def process_odom(self, msg: Odometry):
        """Oppdater robot posisjon og orientering"""
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Oppdater robot orientering (yaw)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Yaw (z-akse rotasjon)
        import math
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.robot_orientation = math.atan2(siny_cosp, cosy_cosp)
        
        # Oppdater komponenter med ny posisjon
        self.robot_memory.update_robot_pose(self.robot_position, self.robot_orientation)
        self.goal_navigator.update_robot_pose(self.robot_position, self.robot_orientation)
        self.big_fire_coordinator.update_state(self.robot_position, self.robot_orientation)

    def handle_big_fire(self, msg: LaserScan):
        """Håndterer Big Fire koordinering"""
        coordinator = self.big_fire_coordinator
        
        self.node.get_logger().info(f'🔥 BIG FIRE HANDLER: Current state = {coordinator.memory.big_fire_state}')
        
        if coordinator.memory.big_fire_state == coordinator.NORMAL:
            # Bestem rolle basert på hvem som oppdaget Big Fire
            if coordinator.memory.big_fire_detected_by_me:
                self.node.get_logger().info('🔥 SCOUT: Jeg oppdaget Big Fire - navigerer til brannen!')
            elif coordinator.memory.big_fire_detected_by_other:
                self.node.get_logger().info('🔥 SUPPORTER: Mottok Big Fire melding - navigerer til brannen!')
                
        elif coordinator.memory.big_fire_state == coordinator.SCOUT_GOING_TO_FIRE:
            # Scout navigerer til Big Fire
            self.node.get_logger().info('🔥 SCOUT: In SCOUT_GOING_TO_FIRE state!')
            target = coordinator.get_target_position()
            if target:
                self.node.get_logger().info(f'🔥 SCOUT: Target position: {target}')
                self.goal_navigator.set_goal(target)
                self.node.get_logger().info('🔥 SCOUT: Calling navigate_to_goal_keep_target()')
                goal_reached = self.goal_navigator.navigate_to_goal_keep_target(msg)
                self.node.get_logger().info(f'🔥 SCOUT: navigate_to_goal_keep_target() returned: {goal_reached}')
                if goal_reached:
                    coordinator.memory.transition_to_scout_waiting()
                    self.node.get_logger().info('🔥 SCOUT: Ankommet Big Fire - VENTER på Supporter!')
                    
        elif coordinator.memory.big_fire_state == coordinator.SCOUT_WAITING:
            # Scout venter på Supporter
            self.wall_follower.stop_robot()
            if not coordinator.memory.i_am_at_fire:
                coordinator.publish_robot_at_fire()
            
            # Sjekk om Supporter har ankommet
            if coordinator.memory.other_robot_at_fire:
                coordinator.memory.transition_to_extinguishing()
                self.node.get_logger().info('🔥 SCOUT: Supporter ankommet - begynner slukking!')
                
        elif coordinator.memory.big_fire_state == coordinator.SUPPORTER_GOING_TO_FIRE:
            # Supporter navigerer til Big Fire
            self.node.get_logger().info('🔥 SUPPORTER: In SUPPORTER_GOING_TO_FIRE state!')
            target = coordinator.get_target_position()
            if target:
                self.node.get_logger().info(f'🔥 SUPPORTER: Target position: {target}')
                self.goal_navigator.set_goal(target)
                self.node.get_logger().info('🔥 SUPPORTER: Calling navigate_to_goal_keep_target()')
                goal_reached = self.goal_navigator.navigate_to_goal_keep_target(msg)
                self.node.get_logger().info(f'🔥 SUPPORTER: navigate_to_goal_keep_target() returned: {goal_reached}')
                if goal_reached:
                    coordinator.memory.transition_to_extinguishing()
                    self.node.get_logger().info('🔥 SUPPORTER: Ankommet Big Fire - begynner slukking!')
                    
        elif coordinator.memory.big_fire_state == coordinator.EXTINGUISHING:
            # Begge roboter slukker brannen sammen
            self.wall_follower.stop_robot()
            if not coordinator.memory.fire_extinguished:
                coordinator.publish_fire_extinguished()
                self.node.get_logger().info('🔥 Brannen slukket! Begge roboter returnerer til normal utforskning.')

    def handle_aruco_detection(self, marker_id: int, position: tuple):
        """Håndterer ArUco marker detection"""
        if marker_id == 4:  # Big Fire
            self.node.get_logger().info(f'🔥 BIG FIRE DETECTED! Calling detect_big_fire({position})')
            self.big_fire_coordinator.detect_big_fire(position)
            self.node.get_logger().info(f'🔥 After detect_big_fire: should_handle_big_fire={self.big_fire_coordinator.should_handle_big_fire()}')
        else:
            # Andre markers - rapporter til scoring system
            self.node.get_logger().info(f'📊 ArUco ID {marker_id} på {position}')
