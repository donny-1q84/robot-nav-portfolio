# Algorithms

## Planning
The planner uses A* on a 4-connected grid. Each move costs 1, and the heuristic
is Manhattan distance, which is admissible for 4-connected grids.

## Control
Pure Pursuit is used with a unicycle model. The controller selects a lookahead
point on the path and computes curvature from the heading error.
