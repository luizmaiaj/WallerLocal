"""
Robot class for navigation simulation.
"""
import math
import random
import numpy as np
from src.environment import Environment
from src.constants import HEIGHT, WIDTH, TURN_ANGLE, VIEW_ANGLE


def normalize_angle(angle: float) -> float:
    """Normalize angle to [0, 360) range."""
    return angle % 360


def calculate_angle_between_points(y1: float, x1: float, y2: float, x2: float) -> float:
    """Calculate angle between two points."""
    dy = y2 - y1
    dx = x2 - x1
    angle = math.degrees(math.atan2(dy, dx))
    
    if dx >= 0:
        angle = 360 - angle
    else:
        angle = 180 - angle
        
    return normalize_angle(angle)


def is_angle_in_range(angle1: float, angle2: float, range_: float) -> bool:
    """Check if angle1 is within range_ degrees of angle2."""
    return abs(int(angle1 - angle2)) <= range_


class Robot:
    def __init__(self, environment: Environment):
        """
        Initialize robot with reference to environment.
        
        Args:
            environment: Environment instance for navigation
        """
        self.env = environment
        self.lin = 0.0  # line position
        self.col = 0.0  # column position
        self.dir = 0    # direction in degrees
        
    def initialize(self):
        """Initialize robot at random valid position."""
        while True:
            self.dir = TURN_ANGLE * random.randint(0, 360 // TURN_ANGLE - 1)
            self.col = random.randint(1, WIDTH - 2)
            self.lin = random.randint(1, HEIGHT - 2)

            if self.env.get_cell(int(self.lin), int(self.col)):
                if random.randint(0, 1):
                    self.col = random.randint(1, WIDTH - 2)
                else:
                    self.lin = random.randint(1, HEIGHT - 2)
                    
            if not self.env.get_cell(int(self.lin), int(self.col)):
                break
                
        self.env.set_cell(int(self.lin), int(self.col), 1)
        
    def walk_front(self):
        """Move robot forward one step if possible."""
        angle_rad = math.radians(self.dir)
        test_lin = self.lin - math.sin(angle_rad)
        test_col = self.col + math.cos(angle_rad)
        
        if not self.env.get_cell(int(test_lin), int(test_col)):
            self.env.set_cell(int(self.lin), int(self.col), 0)
            self.lin = test_lin
            self.col = test_col
            self.env.set_cell(int(self.lin), int(self.col), 1)
            
    def walk_back(self):
        """Move robot backward one step if possible."""
        angle_rad = math.radians(self.dir + 180)
        test_lin = self.lin - math.sin(angle_rad)
        test_col = self.col + math.cos(angle_rad)
        
        if not self.env.get_cell(int(test_lin), int(test_col)):
            self.env.set_cell(int(self.lin), int(self.col), 0)
            self.lin = test_lin
            self.col = test_col
            self.env.set_cell(int(self.lin), int(self.col), 1)
            
    def turn_left(self):
        """Turn robot left by TURN_ANGLE degrees."""
        self.dir = normalize_angle(self.dir + TURN_ANGLE)
            
    def turn_right(self):
        """Turn robot right by TURN_ANGLE degrees."""
        self.dir = normalize_angle(self.dir - TURN_ANGLE)
            
    def align(self, ball_lin: float, ball_col: float):
        """
        Align robot with ball if possible.
        
        Args:
            ball_lin: Ball's line position
            ball_col: Ball's column position
        """
        angle = calculate_angle_between_points(self.lin, self.col, ball_lin, ball_col)
        
        if is_angle_in_range(angle, self.dir, VIEW_ANGLE):
            self.dir = normalize_angle(self.dir)
            if self.env.is_path_clear(self.lin, self.col, angle, 1):
                self.dir = angle
                
    def is_near_wall(self) -> bool:
        """Check if robot is near a wall."""
        return not self.env.is_path_clear(self.lin, self.col, self.dir, 2)
        
    def can_see_ball(self, ball_lin: float, ball_col: float) -> bool:
        """
        Check if ball is visible to robot.
        
        Args:
            ball_lin: Ball's line position
            ball_col: Ball's column position
            
        Returns:
            bool: True if ball is visible, False otherwise
        """
        angle = calculate_angle_between_points(self.lin, self.col, ball_lin, ball_col)
        angle_diff = int(angle - self.dir)
        
        if abs(angle_diff) > VIEW_ANGLE and self.env.is_path_clear(self.lin, self.col, angle, 1):
            return False
        return True
        
    def can_see_and_reach_ball(self, ball_lin: float, ball_col: float, angle: float) -> bool:
        """
        Check if ball is visible and reachable.
        
        Args:
            ball_lin: Ball's line position
            ball_col: Ball's column position
            angle: Viewing angle
            
        Returns:
            bool: True if ball is visible and reachable, False otherwise
        """
        return (abs(int(angle - self.dir)) <= VIEW_ANGLE and 
                self.env.is_path_clear(self.lin, self.col, angle, 1))