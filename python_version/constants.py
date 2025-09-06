"""
Global constants for the robot navigation project.
"""

# Evolution parameters
GENERATIONS = 51           # Number of generations
POPULATION_SIZE = 500     # Population size
CROSSOVER_RATE = 0.7     # 70% of population
MUTATION_RATE = 0.1      # Mutation rate

# Environment parameters
HEIGHT = 200             # Grid height
WIDTH = 200             # Grid width

# Simulation parameters
RUNS = 1                # Number of tests per individual
EXECUTIONS = 2000      # Number of tree executions per test
MAX_TREE_SIZE = 1000   # Limit individual length
TURN_ANGLE = 5         # Robot turn angle in degrees
HIT_DISTANCE = 1       # Distance considered for touch
VIEW_ANGLE = 30        # Local vision angle

# GP parameters
MAX_DEPTH = 17         # Maximum tree depth
MAX_NODES = 100        # Maximum number of nodes
TOURNAMENT_SIZE = 5    # Tournament selection size