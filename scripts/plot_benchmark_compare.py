from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def _read_summary(path: Path) -> list[dict]:
    rows = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(row)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot benchmark comparison.")
    parser.add_argument(
        "--summary-csv",
        type=Path,
        default=Path("reports/benchmark_summary.csv"),
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("docs/assets/benchmark_compare.png"),
    )
    args = parser.parse_args()

    rows = _read_summary(args.summary_csv)
    if not rows:
        print("No summary rows found.")
        return

    planners = [row["global_planner"] for row in rows]
    success_rates = [float(row["success_rate"]) for row in rows]
    collision_rates = [float(row["collision_rate"]) for row in rows]
    avg_steps = [float(row["avg_steps"]) for row in rows]

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))

    axes[0].bar(planners, success_rates, color="#1f77b4", label="Success")
    axes[0].bar(planners, collision_rates, color="#d62728", alpha=0.6, label="Collision")
    axes[0].set_ylim(0.0, 1.0)
    axes[0].set_title("Success vs Collision")
    axes[0].legend()
    for idx, value in enumerate(success_rates):
        axes[0].text(idx, value + 0.03, f"{value:.0%}", ha="center", fontsize=9)

    axes[1].bar(planners, avg_steps, color="#2ca02c")
    axes[1].set_title("Avg Steps (Success)")
    for idx, value in enumerate(avg_steps):
        axes[1].text(idx, value, f"{value:.1f}", ha="center", va="bottom", fontsize=9)

    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
