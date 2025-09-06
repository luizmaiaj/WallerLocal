"""
Environment class for robot navigation.
"""
import numpy as np
import math
from constants import HEIGHT, WIDTH


class Environment:
    def __init__(self):
        """Initialize empty environment grid."""
        self.grid = np.zeros((HEIGHT, WIDTH), dtype=np.int8)
        self.initialize()

    def initialize(self):
        """Initialize environment with borders and obstacles."""
        # Set borders
        self.grid[0, :] = 2
        self.grid[-1, :] = 2
        self.grid[:, 0] = 2
        self.grid[:, -1] = 2

        # Add some obstacles (can be customized)
        # TODO: Add your obstacle pattern here

    def is_path_clear(self, start_lin: float, start_col: float, angle: float, steps: int) -> bool:
        """
        Check if path is clear in the given direction.
        
        Args:
            start_lin: Starting line position
            start_col: Starting column position
            angle: Direction angle in degrees
            steps: Number of steps to check
            
        Returns:
            bool: True if path is clear, False otherwise
        """
        rad_angle = math.radians(angle)
        sin_angle = math.sin(rad_angle)
        cos_angle = math.cos(rad_angle)

        for step in range(steps):
            # Calculate position after step
            lin = int(start_lin + step * sin_angle)
            col = int(start_col + step * cos_angle)

            # Check bounds
            if not (0 <= lin < HEIGHT and 0 <= col < WIDTH):
                return False

            # Check for obstacles
            if self.grid[lin, col] == 2:
                return False

        return True

    def set_cell(self, lin: int, col: int, value: int):
        """Set value of a cell in the grid."""
        if 0 <= lin < HEIGHT and 0 <= col < WIDTH:
            self.grid[lin, col] = value

    def get_cell(self, lin: int, col: int) -> int:
        """Get value of a cell in the grid."""
        if 0 <= lin < HEIGHT and 0 <= col < WIDTH:
            return self.grid[lin, col]
        return 2  # Treat out of bounds as obstacle