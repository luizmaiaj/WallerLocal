# Robot Ball-Following Simulation

## Overview
WallerLocal implements genetic algorithm-based evolution of robot behavior for tracking and following a moving ball in an environment with obstacles. The robot must develop efficient ball-following strategies using only local perception.

## Objective
The robot navigates a matrix environment to reach a ball while:
- 200x200 grid world
- Avoiding multiple fixed obstacles
- Continuously tracking the moving ball with physics-based behavior
- Minimizing movement continuous movement and repositioning
- Adapting to random initial positions

## Robot Capabilities
- Local vision (30-degree cone)
- 5-degree angular turns
- Forward/backward movement
- Ball detection and tracking
- Obstacle avoidance

## Constants
```cpp
#define GENS 51          // Number of generations
#define POPULATION 500   // Population size
#define CROSSING 350     // 70% of population for crossover
#define REPRODUCTION 150 // 30% of population for reproduction
#define HEIGHT 200       // Matrix height
#define WIDTH 200        // Matrix width
#define RUNS 1          // Number of tests per individual
#define EXECUTE 2000    // Number of tree executions per test
#define LIMIT 1000      // Individual complexity limit
#define ANGLE 5         // Robot turning angle
#define HIT_DISTANCE 1  // Ball touch detection distance
#define VIEW_ANGLE 30   // Local vision angle
```

## Fitness Evaluation
The fitness function considers:
- Initial distance between robot and ball
- Distance after each ball touch
- Number of robot steps (movement efficiency)
- Number of successful ball hits

## Functions
- `PROGN3 (3)`: Executes three branches in sequence
- `PROGN2 (2)`: Executes two branches in sequence
- `IFWALL (I)`: Executes left branch if wall detected, right branch otherwise
- `IFBALL (C)`: Executes left branch if ball in sight, right branch otherwise

## Terminal Operations
- `WALKFRONT (F)`: Move forward
- `WALKBACK (B)`: Move backward
- `RIGHT (R)`: Turn right (depends on ANGLE)
- `LEFT (L)`: Turn left (depends on ANGLE)
- `ALIGN (A)`: Orient towards ball (max 30 degrees)

## Key Parameters
- Number of generations: Set by GENS
- Population size: Set by POPULATION
- Reproduction probability: Set by REPRODUCTION
- Crossover probability: Set by CROSSING
- Mutation probability: 0%

## Implementation Notes
- Individual complexity is limited by LIMIT parameter
- Uses local vision system
- Includes ball tracking function
- Features memory-efficient design
- Implements obstacle detection

## Project Comparison
This project is part of a series of three robot evolution simulations:

1. WallerGlobal: Uses continuous movement and global perception for wall-following
2. WallerLocal (this project): Focuses on ball-following with local perception
3. WallerRestricted: Implements discrete movement in a smaller environment

Each variation explores different aspects of evolutionary robotics and navigation strategies.