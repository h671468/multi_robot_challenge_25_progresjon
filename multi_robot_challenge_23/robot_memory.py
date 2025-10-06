#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class RobotMemory:
    """
    Robot Memory - EN ansvar: Kun robotens minne og tilstand
    
    Single Responsibility: Kun å huske robotens tilstand og fase
    """
    
    # Big Fire States
    NORMAL = "NORMAL"
    SCOUT_GOING_TO_FIRE = "SCOUT_GOING_TO_FIRE"
    SCOUT_WAITING = "SCOUT_WAITING"
    SUPPORTER_GOING_TO_FIRE = "SUPPORTER_GOING_TO_FIRE"
    EXTINGUISHING = "EXTINGUISHING"
    
    # Roles
    SCOUT = "SCOUT"
    SUPPORTER = "SUPPORTER"
    
    def __init__(self):
        # Robot position tracking
        self.robot_position = (0.0, 0.0)
        self.robot_orientation = 0.0
        
        # Big Fire state
        self.big_fire_position = None
        self.big_fire_detected_by_me = False
        self.big_fire_detected_by_other = False
        self.other_robot_at_fire = False
        self.i_am_at_fire = False
        self.fire_extinguished = False
        
        # Scout & Supporter roles
        self.my_role = None
        self.big_fire_state = self.NORMAL
        
        # Wall following state
        self.is_turning = False
        
        # Goal navigation state
        self.target_position = None
        self.navigation_active = False

    def set_big_fire_detected_by_me(self, position: tuple):
        """Sett Big Fire oppdaget av denne roboten"""
        # Note: Kan ikke bruke self.node.get_logger() her siden RobotMemory ikke har node reference
        self.big_fire_detected_by_me = True
        self.big_fire_position = position
        self.my_role = self.SCOUT
        self.big_fire_state = self.SCOUT_GOING_TO_FIRE

    def set_big_fire_detected_by_other(self, position: tuple):
        """Sett Big Fire oppdaget av annen robot"""
        self.big_fire_detected_by_other = True
        self.big_fire_position = position
        self.my_role = self.SUPPORTER
        self.big_fire_state = self.SUPPORTER_GOING_TO_FIRE

    def transition_to_scout_waiting(self):
        """Transition til Scout venting"""
        self.big_fire_state = self.SCOUT_WAITING

    def transition_to_extinguishing(self):
        """Transition til slukking"""
        self.big_fire_state = self.EXTINGUISHING

    def transition_to_normal(self):
        """Transition til normal tilstand"""
        self.big_fire_state = self.NORMAL

    def set_other_robot_at_fire(self, value: bool):
        """Sett om annen robot er ved brannen"""
        self.other_robot_at_fire = value

    def set_i_am_at_fire(self, value: bool):
        """Sett om jeg er ved brannen"""
        self.i_am_at_fire = value

    def set_fire_extinguished(self, value: bool):
        """Sett om brannen er slukket"""
        self.fire_extinguished = value

    def set_target_position(self, position: tuple):
        """Sett målposisjon"""
        self.target_position = position
        self.navigation_active = True

    def clear_target_position(self):
        """Fjern målposisjon"""
        self.target_position = None
        self.navigation_active = False

    def set_turning_state(self, is_turning: bool):
        """Sett turning state"""
        self.is_turning = is_turning

    def should_handle_big_fire(self) -> bool:
        """Sjekk om vi skal håndtere Big Fire koordinering"""
        result = (self.big_fire_detected_by_me or 
                  self.big_fire_detected_by_other or 
                  self.big_fire_state != self.NORMAL)
        
        # Debug logging - kan ikke bruke node.get_logger() her
        print(f"🔥 should_handle_big_fire: detected_by_me={self.big_fire_detected_by_me}, detected_by_other={self.big_fire_detected_by_other}, state={self.big_fire_state}, result={result}")
        
        return result

    def is_scout_waiting(self) -> bool:
        """Sjekk om Scout venter"""
        return self.big_fire_state == self.SCOUT_WAITING

    def is_extinguishing(self) -> bool:
        """Sjekk om vi slukker brannen"""
        return self.big_fire_state == self.EXTINGUISHING

    def is_scout_going_to_fire(self) -> bool:
        """Sjekk om Scout går til brannen"""
        return self.big_fire_state == self.SCOUT_GOING_TO_FIRE

    def is_supporter_going_to_fire(self) -> bool:
        """Sjekk om Supporter går til brannen"""
        return self.big_fire_state == self.SUPPORTER_GOING_TO_FIRE

    def get_target_position(self) -> tuple:
        """Hent målposisjon"""
        return self.target_position

    def is_navigation_active(self) -> bool:
        """Sjekk om navigasjon er aktiv"""
        return self.navigation_active

    def is_goal_reached(self) -> bool:
        """Sjekk om mål er nådd"""
        return self.big_fire_state in [self.SCOUT_WAITING, self.EXTINGUISHING]

    def update_robot_pose(self, position: tuple, orientation: float):
        """Oppdater robot posisjon og orientering"""
        self.robot_position = position
        self.robot_orientation = orientation

    def reset_big_fire_state(self):
        """Reset Big Fire state"""
        self.big_fire_position = None
        self.big_fire_detected_by_me = False
        self.big_fire_detected_by_other = False
        self.other_robot_at_fire = False
        self.i_am_at_fire = False
        self.fire_extinguished = False
        self.my_role = None
        self.big_fire_state = self.NORMAL
