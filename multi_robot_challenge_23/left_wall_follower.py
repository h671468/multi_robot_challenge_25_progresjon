#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class LeftWallFollower:
    
    # --- ROBOTENS TILSTANDER  ---
    STATE_TURN_RIGHT = 0      # 0: SVING VENSTRE (Kollisjonsunngåelse/Innvendig hjørne)
    STATE_FOLLOW_WALL = 1     # 1: FØLG VEGGEN (Normal kjøring)
    STATE_TURN_LEFT = 2       # 2: SVING HØYRE (Veggen er borte/Søk)
    
    # --- INNSTILLINGER  ---
    WALL_DISTANCE = 0.5       #  Avstand fra veggen (meter)
    FRONT_THRESHOLD = 0.8     # Avstand som utløser sving VENSTRE
    KP_ANGULAR = 0.5          # Styrke på korrigering for å holde avstand
    LINEAR_SPEED = 0.3        # Fart fremover (m/s)
    MAX_RANGE = 3.5           
    TURN_RIGHT_SPEED_ANG = 0.8 # Svingehastighet for venstresving (økt)
    TURN_RIGHT_SPEED_LIN = 0.2 # Høyere fart fremover under sving
    
    # Grense for å kontrollere maks svingehastighet
    MAX_ANGULAR_Z = 0.8 
    
    def __init__(self, node_ref: Node):
        self.node = node_ref
        self.node.get_logger().info('LeftWallFollower-klassen er klar.')
        
        self.state = self.STATE_FOLLOW_WALL 
        self.regions = {}
        self.is_active = True
        
        # Timer for svinger
        self.turn_start_time = None
        self.turn_duration = 2.0  # Sving i 2 sekunder 

    def process_scan(self, msg: LaserScan):
        if not self.is_active:
            self.node.publish_twist(0.0, 0.0)
            return

        ranges = np.array(msg.ranges)
        
        # FIKSING AV DATA: Erstatter ugyldige avstander med MAX_RANGE
        ranges[np.isinf(ranges)] = self.MAX_RANGE
        ranges[ranges == 0.0] = self.MAX_RANGE
        
        n = len(ranges)

        def safe_min(slice_array):
            return np.min(slice_array) if len(slice_array) > 0 else self.MAX_RANGE

        # --- DELER SENSOR-DATA INN I REGIONER FOR VENSTRE VEGGFØLGING ---
        self.regions = {
            'front': min(safe_min(ranges[0:int(n*0.05)]), safe_min(ranges[int(n*0.95):])), 
            'left': safe_min(ranges[int(n*0.10):int(n*0.20)]), 
            'back_left': safe_min(ranges[int(n*0.20):int(n*0.25)]),
        }

        self.take_action()

        self.node.get_logger().info(
            f"Venstre veggfølger - front={self.regions['front']:.2f}, left={self.regions['left']:.2f}, back_left={self.regions['back_left']:.2f}"
        )

    def process_odom(self, msg):
        """Prosesserer odometri-data (kun for kompatibilitet)"""
        # Veggfølgeren bruker ikke odometri-data, men metoden må eksistere
        pass

    def decide_state(self):
        d_front = self.regions.get('front', self.MAX_RANGE)
        d_left = self.regions.get('left', self.MAX_RANGE)
        d_back_left = self.regions.get('back_left', self.MAX_RANGE)
        
        # Hvis vi allerede svinger, fortsett til tiden er ute
        if self.state == self.STATE_TURN_RIGHT and self.turn_start_time is not None:
            elapsed = (self.node.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
            if elapsed < self.turn_duration:
                return self.STATE_TURN_RIGHT
            else:
                # Svingen er ferdig, tilbake til normal
                self.turn_start_time = None
                return self.STATE_FOLLOW_WALL
        
        # 1. VEGG FORAN? (Start sving)
        if d_front < self.FRONT_THRESHOLD:
            if self.turn_start_time is None:
                self.turn_start_time = self.node.get_clock().now()
            return self.STATE_TURN_RIGHT
        
        # 2. VEGGEN BORTE? (Utvendig hjørne/åpent rom)
        elif d_left > (self.WALL_DISTANCE + 0.3) and d_back_left > (self.WALL_DISTANCE + 0.3):
            return self.STATE_TURN_LEFT
            
        # 3. NORMAL KJØRING
        else:
            return self.STATE_FOLLOW_WALL

    def take_action(self):
        self.state = self.decide_state()
        linear_x = 0.0
        angular_z = 0.0

        state_str = {0: "TURN_RIGHT", 1: "FOLLOW_WALL", 2: "TURN_LEFT"}[self.state]
        self.node.get_logger().info(f"Venstre veggfølger - State: {state_str}")

        # SVING VENSTRE (Innvendig hjørne)
        if self.state == self.STATE_TURN_RIGHT:
            linear_x = 0.0  # Stopp fremover under sving
            angular_z = self.TURN_RIGHT_SPEED_ANG 

        # --- FORBEDRING FOR UTGANG AV HJØRNE (SØK) ---
        elif self.state == self.STATE_TURN_LEFT:
            linear_x = self.LINEAR_SPEED * 0.8  # Høyere fart for å komme ut av hjørne
            angular_z = -0.3  # Større sving for å finne vegg raskere 

        # FØLG VEGGEN (P-kontroll)
        elif self.state == self.STATE_FOLLOW_WALL:
            d_left = self.regions['left']
            
            # Avstandsfeil
            error = d_left - self.WALL_DISTANCE
            
            # P-kontroll (sving venstre når for nær veggen, høyre når for langt fra)
            angular_z = -self.KP_ANGULAR * error
            
            # Begrens svingen til +/- 0.5
            angular_z = max(min(angular_z, self.MAX_ANGULAR_Z), -self.MAX_ANGULAR_Z) 
            linear_x = self.LINEAR_SPEED

        self.node.get_logger().info(
            f"Venstre veggfølger - Cmd_vel: linear.x={linear_x:.2f}, angular.z={angular_z:.2f}"
        )

        self.node.publish_twist(linear_x, angular_z)
