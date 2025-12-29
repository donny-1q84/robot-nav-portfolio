# Algorithms

## Planning
The global planner supports:
- **A*** (Manhattan heuristic on a 4-connected grid)
- **Dijkstra** (A* with zero heuristic)
- **Theta\*** (any-angle planning with line-of-sight shortcuts)

## Costmap & Collision
Obstacles can be inflated by a configurable radius to create a conservative
costmap. Collision checks treat any pose that maps to an inflated cell as a
collision and also consider out-of-bounds positions as collisions.

## Local Costmap
Local planning can use a rolling window that masks obstacles outside a radius,
mirroring a limited sensor range. This reduces compute and keeps DWA focused on
nearby hazards.

## Local Planning
The local planner uses a simplified Dynamic Window Approach (DWA). It samples
constant (v, omega) commands, rolls out short trajectories, rejects collisions,
and scores candidates based on distance to the goal, distance to the global path,
and obstacle clearance.

## Dynamic Obstacles & Replanning
Dynamic obstacles move with simple velocities and bounce at map boundaries.
Their occupied cells are overlaid onto the costmap each step. The planner
replans when the current path intersects the updated costmap or after a fixed
step interval.

## Localization
An EKF estimates the robot pose using noisy odometry (control inputs) and noisy
position measurements. The controller can use the estimated pose for planning
and tracking, while the true pose is used for visualization and evaluation.

## Control
Pure Pursuit is used with a unicycle model. The controller selects a lookahead
point on the path and computes curvature from the heading error.
