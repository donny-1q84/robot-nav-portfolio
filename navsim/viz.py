from __future__ import annotations

from typing import List, Tuple

import matplotlib.pyplot as plt

from .map import GridMap


Point = Tuple[float, float]
Pose = Tuple[float, float, float]


def plot_scene(
    grid: GridMap,
    path: List[Point],
    poses: List[Pose],
    start: Tuple[int, int],
    goal: Tuple[int, int],
    out_path: str,
) -> None:
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.imshow(grid.grid, cmap="Greys", origin="lower")

    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, color="#1f77b4", linewidth=2, label="Path")

    if poses:
        tx = [p[0] for p in poses]
        ty = [p[1] for p in poses]
        ax.plot(tx, ty, color="#ff7f0e", linewidth=2, label="Trajectory")

    ax.scatter([start[0]], [start[1]], color="#2ca02c", s=80, label="Start")
    ax.scatter([goal[0]], [goal[1]], color="#d62728", s=80, label="Goal")

    ax.set_xlim(-0.5, grid.width - 0.5)
    ax.set_ylim(-0.5, grid.height - 0.5)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def render_gif(
    grid: GridMap,
    path: List[Point],
    poses: List[Pose],
    start: Tuple[int, int],
    goal: Tuple[int, int],
    out_path: str,
    step: int = 3,
) -> None:
    import imageio.v2 as imageio

    frames = []
    for i in range(1, len(poses) + 1, step):
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.imshow(grid.grid, cmap="Greys", origin="lower")
        if path:
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            ax.plot(xs, ys, color="#1f77b4", linewidth=2)
        tx = [p[0] for p in poses[:i]]
        ty = [p[1] for p in poses[:i]]
        ax.plot(tx, ty, color="#ff7f0e", linewidth=2)
        ax.scatter([start[0]], [start[1]], color="#2ca02c", s=80)
        ax.scatter([goal[0]], [goal[1]], color="#d62728", s=80)
        ax.set_xlim(-0.5, grid.width - 0.5)
        ax.set_ylim(-0.5, grid.height - 0.5)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.canvas.draw()
        frame = imageio.imread(fig.canvas.buffer_rgba())
        frames.append(frame)
        plt.close(fig)

    imageio.mimsave(out_path, frames, fps=12)
