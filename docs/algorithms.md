# Algorithms

## Planning
The planner uses A* on a 4-connected grid. Each move costs 1, and the heuristic
is Manhattan distance, which is admissible for 4-connected grids.

## Costmap & Collision
Obstacles can be inflated by a configurable radius to create a conservative
costmap. Collision checks treat any pose that maps to an inflated cell as a
collision and also consider out-of-bounds positions as collisions.

## Local Planning
The local planner uses a simplified Dynamic Window Approach (DWA). It samples
constant (v, omega) commands, rolls out short trajectories, rejects collisions,
and scores candidates based on distance to the goal, distance to the global path,
and obstacle clearance.

## Control
Pure Pursuit is used with a unicycle model. The controller selects a lookahead
point on the path and computes curvature from the heading error.
