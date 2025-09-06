"""
Visualization utilities for robot navigation.
"""
import os
import subprocess
from PIL import Image
import numpy as np
from typing import Tuple
from src.constants import HEIGHT, WIDTH
from src.environment import Environment
from src.robot import Robot
from src.gp.engine import BallData


class TrackingVisualizer:
    """Handles visualization of robot and ball paths."""
    
    def __init__(self):
        """Initialize tracking arrays."""
        self.best_track = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        self.robot_track = np.zeros((HEIGHT, WIDTH), dtype=np.int32)
        self.ball_track = np.zeros((HEIGHT, WIDTH), dtype=np.int32)
        
    def update_best_track(self, env: Environment, robot: Robot, ball: BallData):
        """Update visualization with current state."""
        # Clear track
        self.best_track.fill(0)
        
        # Draw borders and obstacles
        for lin in range(HEIGHT):
            for col in range(WIDTH):
                if (lin == 0 or lin == HEIGHT-1 or 
                    col == 0 or col == WIDTH-1 or 
                    env.get_cell(lin, col) == 2):
                    self.best_track[lin, col] = [255, 255, 255]
        
        # Draw robot path
        for lin in range(HEIGHT):
            for col in range(WIDTH):
                if self.robot_track[lin, col] > 0:
                    if self.robot_track[lin, col] < 20:
                        self.best_track[lin, col] = [255, 255, 0]  # Yellow
                    else:
                        self.best_track[lin, col] = [255, 0, 0]    # Red
        
        # Draw ball path
        for lin in range(HEIGHT):
            for col in range(WIDTH):
                if self.ball_track[lin, col] == 1:
                    self.best_track[lin, col] = [0, 255, 0]  # Green
                    
    def save_track(self, generation: int):
        """Save current track as an image."""
        print("\n\t\tSaving visualization...")
        
        # Create paths directory if it doesn't exist
        os.makedirs("paths", exist_ok=True)
        
        # Save as PPM
        ppm_path = f"paths/caminho{generation:03d}.ppm"
        with open(ppm_path, 'w') as f:
            f.write(f"P3\n{WIDTH} {HEIGHT}\n255\n")
            for lin in range(HEIGHT):
                for col in range(WIDTH):
                    f.write(f"{self.best_track[lin,col,0]} "
                           f"{self.best_track[lin,col,1]} "
                           f"{self.best_track[lin,col,2]}   ")
                f.write("\n")
                
        # Convert to GIF using ImageMagick
        gif_path = f"paths/caminho{generation:03d}.gif"
        try:
            subprocess.run(["magick", ppm_path, gif_path], check=True)
            os.remove(ppm_path)  # Clean up PPM file
        except subprocess.CalledProcessError:
            print("Warning: Failed to convert PPM to GIF. Is ImageMagick installed?")
            
    def update_robot_position(self, lin: int, col: int, step: int):
        """Update robot position in tracking."""
        if 0 <= lin < HEIGHT and 0 <= col < WIDTH:
            self.robot_track[lin, col] = step
            
    def update_ball_position(self, lin: int, col: int):
        """Update ball position in tracking."""
        if 0 <= lin < HEIGHT and 0 <= col < WIDTH:
            self.ball_track[lin, col] = 1
            
    def clear_tracks(self):
        """Clear all tracking arrays."""
        self.robot_track.fill(0)
        self.ball_track.fill(0)