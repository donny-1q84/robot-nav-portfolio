# Benchmark

The benchmark runner samples random start/goal pairs on the grid, plans with A*,
simulates execution, and writes per-trial metrics to CSV.

## Metrics
- `plan_found`: 1 if A* found a path.
- `success`: 1 if the robot reaches the goal within tolerance.
- `steps`: number of simulation steps taken.
- `path_length`: length of the global path.
- `traj_length`: length of the executed trajectory.
- `final_distance`: distance from final pose to goal.
- `collision`: 1 if the trajectory intersects the inflated costmap.
- `elapsed_ms`: total time for planning + simulation.

## Usage
```bash
navsim-benchmark --trials 50 --csv reports/benchmark.csv
```

## Plot
```bash
.venv/bin/python scripts/plot_benchmark.py --csv reports/benchmark.csv \
  --out docs/assets/benchmark_summary.png
```
