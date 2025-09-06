"""
GP engine implementation using DEAP framework.
"""
from dataclasses import dataclass
from typing import List, Tuple, Optional
import random
import numpy as np
from deap import base, creator, tools, gp

from .primitives import RobotCommand, RobotNodeValue
from ..robot import Robot
from ..environment import Environment
from ..constants import (
    HEIGHT, WIDTH, EXECUTIONS, RUNS, HIT_DISTANCE,
    POPULATION_SIZE, GENERATIONS, CROSSOVER_RATE, MUTATION_RATE,
    TOURNAMENT_SIZE, MAX_DEPTH
)


@dataclass
class BallData:
    """Ball state data."""
    dir: int = 0
    lin: float = 0.0
    col: float = 0.0


class RobotEvaluator:
    """Evaluator class that executes robot commands."""
    
    def __init__(self, robot: Robot, ball: BallData):
        """Initialize evaluator with robot and ball references."""
        self.robot = robot
        self.ball = ball
        
    def execute_command(self, cmd: RobotNodeValue):
        """Execute a single robot command."""
        if cmd.cmd == RobotCommand.WALKFRONT:
            self.robot.walk_front()
        elif cmd.cmd == RobotCommand.WALKBACK:
            self.robot.walk_back()
        elif cmd.cmd == RobotCommand.LEFT:
            self.robot.turn_left()
        elif cmd.cmd == RobotCommand.RIGHT:
            self.robot.turn_right()
        elif cmd.cmd == RobotCommand.ALIGN:
            self.robot.align(self.ball.lin, self.ball.col)
            
    def check_condition(self, cmd: RobotNodeValue) -> bool:
        """Check condition for if-nodes."""
        if cmd.cmd == RobotCommand.IFWALL:
            return self.robot.is_near_wall()
        elif cmd.cmd == RobotCommand.IFBALL:
            return self.robot.can_see_ball(self.ball.lin, self.ball.col)
        return False


class GPEngine:
    """Main GP engine using DEAP."""
    
    def __init__(self, env: Environment, robot: Robot, ball: BallData):
        """Initialize GP engine with environment and actors."""
        self.env = env
        self.robot = robot
        self.ball = ball
        self.evaluator = RobotEvaluator(robot, ball)
        
        # Set up DEAP
        self._setup_deap()
        
    def _setup_deap(self):
        """Configure DEAP components."""
        # Create fitness and individual classes
        creator.create("FitnessMax", base.Fitness, weights=(1.0,))
        creator.create("Individual", gp.PrimitiveTree, fitness=creator.FitnessMax)
        
        # Create primitive set
        self.pset = gp.PrimitiveSet("ROBOT", 0)
        
        # Add functions
        self.pset.addPrimitive(self._prog3, 3, name="PROGN3")
        self.pset.addPrimitive(self._prog2, 2, name="PROGN2")
        self.pset.addPrimitive(self._if_wall, 2, name="IFWALL")
        self.pset.addPrimitive(self._if_ball, 2, name="IFBALL")
        
        # Add terminals
        self.pset.addTerminal(RobotCommand.WALKFRONT, name="WALKFRONT")
        self.pset.addTerminal(RobotCommand.WALKBACK, name="WALKBACK")
        self.pset.addTerminal(RobotCommand.LEFT, name="LEFT")
        self.pset.addTerminal(RobotCommand.RIGHT, name="RIGHT")
        self.pset.addTerminal(RobotCommand.ALIGN, name="ALIGN")
        
        # Create toolbox
        self.toolbox = base.Toolbox()
        self.toolbox.register("expr", gp.genHalfAndHalf, pset=self.pset, min_=1, max_=4)
        self.toolbox.register("individual", tools.initIterate, creator.Individual, self.toolbox.expr)
        self.toolbox.register("population", tools.initRepeat, list, self.toolbox.individual)
        self.toolbox.register("evaluate", self._evaluate_individual)
        self.toolbox.register("select", tools.selTournament, tournsize=TOURNAMENT_SIZE)
        self.toolbox.register("mate", gp.cxOnePoint)
        self.toolbox.register("expr_mut", gp.genFull, min_=0, max_=2)
        self.toolbox.register("mutate", gp.mutUniform, expr=self.toolbox.expr_mut, pset=self.pset)
        
    # DEAP primitive implementations
    def _prog2(self, x1, x2):
        """Execute two commands in sequence."""
        if isinstance(x1, RobotCommand):
            self.evaluator.execute_command(RobotNodeValue(x1))
        if isinstance(x2, RobotCommand):
            self.evaluator.execute_command(RobotNodeValue(x2))
        return x2
    
    def _prog3(self, x1, x2, x3):
        """Execute three commands in sequence."""
        if isinstance(x1, RobotCommand):
            self.evaluator.execute_command(RobotNodeValue(x1))
        if isinstance(x2, RobotCommand):
            self.evaluator.execute_command(RobotNodeValue(x2))
        if isinstance(x3, RobotCommand):
            self.evaluator.execute_command(RobotNodeValue(x3))
        return x3
    
    def _if_wall(self, x1, x2):
        """Execute x1 if near wall, x2 otherwise."""
        if self.evaluator.check_condition(RobotNodeValue(RobotCommand.IFWALL)):
            if isinstance(x1, RobotCommand):
                self.evaluator.execute_command(RobotNodeValue(x1))
            return x1
        if isinstance(x2, RobotCommand):
            self.evaluator.execute_command(RobotNodeValue(x2))
        return x2
    
    def _if_ball(self, x1, x2):
        """Execute x1 if ball visible, x2 otherwise."""
        if self.evaluator.check_condition(RobotNodeValue(RobotCommand.IFBALL)):
            if isinstance(x1, RobotCommand):
                self.evaluator.execute_command(RobotNodeValue(x1))
            return x1
        if isinstance(x2, RobotCommand):
            self.evaluator.execute_command(RobotNodeValue(x2))
        return x2
    
    def _evaluate_individual(self, individual) -> Tuple[float]:
        """Evaluate fitness of an individual."""
        program = gp.compile(individual, self.pset)
        total_fitness = 0.0
        
        for _ in range(RUNS):
            # Initialize for this run
            self.env.initialize()
            self.robot.initialize()
            
            # Initialize ball position
            while True:
                self.ball.col = random.randint(1, WIDTH-2)
                self.ball.lin = random.randint(1, HEIGHT-2)
                if not self.env.get_cell(int(self.ball.lin), int(self.ball.col)):
                    break
            self.env.set_cell(int(self.ball.lin), int(self.ball.col), 1)
            
            # Get initial distance
            initial_distance = np.sqrt(
                (self.ball.lin - self.robot.lin)**2 + 
                (self.ball.col - self.robot.col)**2
            )
            
            # Run simulation
            hits = 0
            unfit = 0
            last_hit_step = 0
            
            for step in range(EXECUTIONS):
                # Execute program
                try:
                    program()
                except Exception:
                    break
                    
                # Check for ball hit
                hit_distance = np.sqrt(
                    (self.ball.lin - self.robot.lin)**2 + 
                    (self.ball.col - self.robot.col)**2
                )
                
                if hit_distance <= HIT_DISTANCE:
                    hits += 1
                    unfit += (step - last_hit_step) / initial_distance
                    last_hit_step = step
                    
                    # Move ball after hit
                    self.ball.dir = self.robot.dir
                    ball_movements = 40
                    
                    while ball_movements > 0:
                        ball_movements -= 1
                        angle = np.radians(self.ball.dir)
                        test_lin = self.ball.lin - (2 * np.sin(angle))
                        test_col = self.ball.col + (2 * np.cos(angle))
                        
                        # Check bounds and obstacles
                        if (test_lin < 0 or test_lin >= HEIGHT or
                            test_col < 0 or test_col >= WIDTH or
                            self.env.get_cell(int(test_lin), int(test_col))):
                            # Bounce
                            self.ball.dir = (self.ball.dir + 180) % 360
                            test_lin = self.ball.lin - (2 * np.sin(np.radians(self.ball.dir)))
                            test_col = self.ball.col + (2 * np.cos(np.radians(self.ball.dir)))
                            
                        # Update ball position
                        self.env.set_cell(int(self.ball.lin), int(self.ball.col), 0)
                        self.ball.lin = test_lin
                        self.ball.col = test_col
                        self.env.set_cell(int(self.ball.lin), int(self.ball.col), 1)
            
            # Calculate fitness for this run
            run_fitness = 1500 * hits - unfit
            total_fitness += run_fitness
            
        return (total_fitness / RUNS,)
    
    def evolve(self, population_size: int = POPULATION_SIZE, 
              generations: int = GENERATIONS,
              crossover_prob: float = CROSSOVER_RATE, 
              mutation_prob: float = MUTATION_RATE) -> Tuple[List, tools.Logbook]:
        """
        Evolve population for specified number of generations.
        
        Args:
            population_size: Size of population
            generations: Number of generations to evolve
            crossover_prob: Probability of crossover
            mutation_prob: Probability of mutation
            
        Returns:
            Tuple of (final population, evolution statistics)
        """
        # Create initial population
        pop = self.toolbox.population(n=population_size)
        hof = tools.HallOfFame(1)  # Keep track of best individual
        
        # Statistics setup
        stats = tools.Statistics(lambda ind: ind.fitness.values)
        stats.register("avg", np.mean)
        stats.register("std", np.std)
        stats.register("min", np.min)
        stats.register("max", np.max)
        
        # Run evolution
        pop, logbook = algorithms.eaSimple(pop, self.toolbox, 
                                         crossover_prob, mutation_prob, generations,
                                         stats=stats, halloffame=hof, verbose=True)
        
        return pop, logbook