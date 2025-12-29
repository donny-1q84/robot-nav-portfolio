from __future__ import annotations

import argparse
import csv
import statistics
from pathlib import Path

import matplotlib.pyplot as plt


def _read_rows(path: Path) -> list[dict]:
    rows = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            converted: dict = {}
            for key, value in row.items():
                if value is None:
                    continue
                try:
                    if key in {"plan_found", "success", "collision", "steps"}:
                        converted[key] = int(float(value))
                    else:
                        converted[key] = float(value)
                except ValueError:
                    converted[key] = value
            rows.append(converted)
    return rows


def _mean(values: list[float]) -> float:
    return statistics.mean(values) if values else 0.0


def _summarize(rows: list[dict]) -> dict:
    trials = len(rows)
    successes = [row for row in rows if row["success"] == 1]
    return {
        "trials": trials,
        "plan_success_rate": _mean([row["plan_found"] for row in rows]),
        "success_rate": _mean([row["success"] for row in rows]),
        "collision_rate": _mean([row["collision"] for row in rows]),
        "avg_steps": _mean([row["steps"] for row in successes]),
        "avg_path_length": _mean([row["path_length"] for row in successes]),
        "avg_traj_length": _mean([row["traj_length"] for row in successes]),
        "avg_final_distance": _mean([row["final_distance"] for row in rows]),
        "avg_elapsed_ms": _mean([row["elapsed_ms"] for row in rows]),
    }


def _annotate_bars(ax, values, fmt="{:.2f}") -> None:
    for idx, value in enumerate(values):
        ax.text(idx, value, fmt.format(value), ha="center", va="bottom", fontsize=8)


def main() -> None:
    parser = argparse.ArgumentParser(description="Render benchmark summary plot.")
    parser.add_argument("--csv", type=Path, default=Path("reports/benchmark.csv"))
    parser.add_argument(
        "--out", type=Path, default=Path("docs/assets/benchmark_summary.png")
    )
    args = parser.parse_args()

    rows = _read_rows(args.csv)
    if not rows:
        print("No rows found in CSV.")
        return

    summary = _summarize(rows)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))

    rates = [
        summary["plan_success_rate"],
        summary["success_rate"],
        summary["collision_rate"],
    ]
    rate_labels = ["Plan", "Success", "Collision"]
    axes[0].bar(rate_labels, rates, color=["#2ca02c", "#1f77b4", "#d62728"])
    axes[0].set_ylim(0.0, 1.0)
    axes[0].set_ylabel("Rate")
    axes[0].set_title("Rates")
    for idx, value in enumerate(rates):
        axes[0].text(idx, value + 0.03, f"{value:.0%}", ha="center", fontsize=9)

    averages = [
        summary["avg_steps"],
        summary["avg_path_length"],
        summary["avg_traj_length"],
        summary["avg_final_distance"],
        summary["avg_elapsed_ms"],
    ]
    avg_labels = ["Steps", "Path len", "Traj len", "Final dist", "Elapsed ms"]
    axes[1].bar(avg_labels, averages, color="#9467bd")
    axes[1].set_title("Averages")
    axes[1].tick_params(axis="x", rotation=20)
    _annotate_bars(axes[1], averages)

    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
