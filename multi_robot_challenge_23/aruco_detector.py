#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from rclpy.node import Node
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose

class ArUcoDetector:
    """
    ArUco marker detection - EN ansvar: ArUco marker håndtering
    
    Single Responsibility: Kun ArUco detection og rapportering
    """
    
    def __init__(self, node_ref: Node, callback=None):
        self.node = node_ref
        
        # ArUco tracking
        self.detected_markers = {}  # {marker_id: (x, y, timestamp)}
        self.current_marker_id = None
        
        # Callback for ArUco detection
        self.aruco_callback = callback
        
        # Setup subscribers
        self.setup_subscribers()
        
        self.node.get_logger().info('📊 ArUcoDetector initialisert')

    def setup_subscribers(self):
        """Sett opp ArUco marker detection subscribers"""
        # Subscriber for ArUco marker ID (riktig topic basert på dokumentasjon)
        self.marker_id_sub = self.node.create_subscription(
            Int64, 'marker_id', self.marker_id_callback, 10
        )
        
        # Subscriber for ArUco marker pose i map koordinater (riktig topic basert på dokumentasjon)
        self.marker_pose_sub = self.node.create_subscription(
            Pose, 'marker_map_pose', self.marker_pose_callback, 10
        )

    def marker_id_callback(self, msg: Int64):
        """Håndterer ArUco marker ID detection"""
        marker_id = msg.data
        self.node.get_logger().info(f'📊 ArUco ID {marker_id} oppdaget!')
        self.current_marker_id = marker_id

    def marker_pose_callback(self, msg: Pose):
        """Håndterer ArUco marker pose i map koordinater"""
        if self.current_marker_id is not None:
            position = (
                msg.position.x,
                msg.position.y,
                msg.position.z
            )
            
            self.node.get_logger().info(
                f'📍 ArUco ID {self.current_marker_id} posisjon: '
                f'x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}'
            )
            
            # Lagre detektert merke
            self.detected_markers[self.current_marker_id] = (
                position[0], position[1], self.node.get_clock().now()
            )
            
            # Rapporter ArUco-merke
            self.report_aruco_marker(self.current_marker_id, position)
            
            # Rydd opp
            self.current_marker_id = None

    def report_aruco_marker(self, marker_id: int, position: tuple):
        """Rapporter ArUco-merke"""
        self.node.get_logger().info(f'📊 ArUco ID {marker_id} oppdaget på {position}')
        
        # Kaller callback hvis den er satt (for Big Fire koordinering)
        if self.aruco_callback:
            self.node.get_logger().info(f'📊 Calling aruco_callback for marker_id={marker_id}')
            self.aruco_callback(marker_id, position)
        else:
            self.node.get_logger().warn(f'📊 No aruco_callback set for marker_id={marker_id}')
        
        # Spesialhåndtering for Big Fire (ID 4)
        if marker_id == 4:
            self.node.get_logger().info('🔥 BIG FIRE OPPDAGET!')
            return marker_id, position
        
        # TODO: Kall scoring service for andre markers
        self.node.get_logger().info(f'📊 RAPPORTERER: ArUco ID {marker_id} på posisjon {position}')
        return marker_id, position

    def get_detected_markers(self) -> dict:
        """Hent alle detekterte markers"""
        return self.detected_markers

    def has_marker(self, marker_id: int) -> bool:
        """Sjekk om marker er detektert"""
        return marker_id in self.detected_markers
