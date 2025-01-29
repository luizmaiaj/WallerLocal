# WallerLocal - Local Vision Ball-Following Robot Evolution

## Overview
WallerLocal implements genetic algorithm-based evolution of robot behavior for tracking and following a moving ball in an environment with obstacles. The robot must develop efficient ball-following strategies using only local perception.

## Technical Specifications

### Environment
- 200x200 grid world
- Multiple fixed obstacles
- Moving ball with physics-based behavior
- Continuous movement and positioning

### Robot Capabilities
- Local vision (30-degree cone)
- 5-degree angular turns
- Forward/backward movement
- Ball detection and tracking
- Obstacle avoidance

### Genetic Algorithm Parameters
- Population size: 500 individuals
- Generations: 51
- Selection ratio: 70% crossover, 30% reproduction
- Maximum individual complexity: 1000 nodes
- Single evaluation run per individual
- 2000 execution steps per run

### Available Functions
- `PROGN3`: Executes three branches sequentially
- `PROGN2`: Executes two branches sequentially
- `IFWALL`: Conditional wall detection branching
- `IFBALL`: Conditional ball detection branching

### Terminal Operations
- `WALKFRONT`: Move forward
- `WALKBACK`: Move backward
- `RIGHT`: Turn right 5 degrees
- `LEFT`: Turn left 5 degrees
- `ALIGN`: Orient towards ball (max 30 degrees)

### Fitness Function
Based on:
- Ball hits achieved
- Initial distance to ball
- Movement efficiency
- Calculated as: `1500 * ball_hits - unfit`

## Project Comparison
This project is part of a series of three robot evolution simulations:

1. WallerGlobal: Uses continuous movement and global perception for wall-following
2. WallerLocal (this project): Focuses on ball-following with local perception
3. WallerRestricted: Implements discrete movement in a smaller environment

Each variation explores different aspects of evolutionary robotics and navigation strategies.