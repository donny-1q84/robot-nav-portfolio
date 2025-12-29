# Robot Navigation Portfolio Demo

This project is a minimal closed-loop 2D navigation demo:
- Grid map with obstacles
- A* path planning
- Pure Pursuit control on a unicycle model
- Matplotlib visualization (PNG, optional GIF)

## Quick Start
```bash
python -m scripts.run_demo --start 0,0 --goal 9,9 --png output.png
```

Optional GIF:
```bash
python -m scripts.run_demo --gif output.gif
```

## Parameters
```
--start x,y     Start grid cell
--goal x,y      Goal grid cell
--png path      Output PNG path (default: output.png)
--gif path      Optional GIF path
--lookahead     Pure pursuit lookahead (default: 0.8)
--speed         Linear speed (default: 0.8)
```

## Design Notes
- **Grid map**: hard-coded demo map in `navsim/map.py`.
- **Planner**: 4-connected A* with Manhattan heuristic.
- **Controller**: Pure Pursuit with unicycle kinematics.
- **Visualization**: map, planned path, and executed trajectory.

## Trade-offs
- No dynamic obstacle handling or replanning.
- Control and planning are decoupled for clarity.
- Uses a simple grid and kinematics to keep the demo focused.

## Tests
```bash
pytest
```
