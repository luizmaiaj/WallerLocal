"""
Main script for robot navigation GP evolution.
"""
import os
import time
import logging
from datetime import datetime
import numpy as np

from environment import Environment
from robot import Robot
from gp.engine import GPEngine, BallData
from visualization.tracking import TrackingVisualizer
from constants import GENERATIONS, POPULATION_SIZE, CROSSOVER_RATE, MUTATION_RATE

# Setup logging
logging.basicConfig(level=logging.INFO,
                   format='%(asctime)s - %(levelname)s - %(message)s')


def count_existing_files(base_name: str, extension: str) -> int:
    """Count number of existing files with given pattern."""
    count = 0
    while True:
        path = f"{base_name}{count:03d}{extension}"
        if not os.path.exists(path):
            break
        count += 1
    return count


def setup_directories():
    """Create necessary directories if they don't exist."""
    os.makedirs("data", exist_ok=True)
    os.makedirs("paths", exist_ok=True)
    os.makedirs("robots", exist_ok=True)


def main():
    """Main execution function."""
    # Create directories
    setup_directories()
    
    # Initialize components
    env = Environment()
    robot = Robot(env)
    ball = BallData()
    visualizer = TrackingVisualizer()
    
    # Initialize GP engine
    gp_engine = GPEngine(env, robot, ball)
    
    # Setup data logging
    data_file_count = count_existing_files("data/data", ".txt")
    data_path = f"data/data{data_file_count}.txt"
    
    with open(data_path, "w") as data_file:
        # Write header
        data_file.write("ROBO SEGUIDOR v2.0 (Python Version)\n")
        data_file.write("GENERATION\tAVERAGE\t\tBEST\n")
        
        # Record start time
        start_time = time.time()
        
        # Evolution
        logging.info("Starting evolution...")
        population, stats = gp_engine.evolve(
            population_size=POPULATION_SIZE,
            generations=GENERATIONS,
            crossover_prob=CROSSOVER_RATE,
            mutation_prob=MUTATION_RATE
        )
        
        # Save results
        for gen in range(GENERATIONS):
            record = stats.select("avg", "max")[gen]
            data_file.write(f"{gen}\t{record['avg']:.2f}\t{record['max']:.2f}\n")
            
            # Update and save visualization
            visualizer.update_best_track(env, robot, ball)
            visualizer.save_track(gen)
            
        # Save final population
        logging.info("Saving final population...")
        robot_file_count = count_existing_files("robots/rb", "tr.txt")
        
        for i, individual in enumerate(population[:100]):
            filename = f"robots/rb{(robot_file_count + i) % 1000:03d}tr.txt"
            with open(filename, "w") as robot_file:
                robot_file.write(f"{str(individual)}\n")
                robot_file.write(f"LENGTH = {len(individual)}\n")
                robot_file.write(f"FITNESS = {individual.fitness.values[0]}\n")
                
        # Calculate runtime
        end_time = time.time()
        runtime = int(end_time - start_time)
        hours = runtime // 3600
        minutes = (runtime % 3600) // 60
        seconds = runtime % 60
        
        logging.info(f"Total runtime: {hours:02d}:{minutes:02d}:{seconds:02d}")


if __name__ == "__main__":
    main()