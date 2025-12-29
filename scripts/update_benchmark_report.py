from __future__ import annotations

import argparse
import csv
from pathlib import Path


def _read_summary(path: Path) -> list[dict]:
    rows = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(row)
    return rows


def _table(rows: list[dict]) -> str:
    headers = [
        "Planner",
        "Plan Success",
        "Success",
        "Collision",
        "Avg Steps",
        "Avg Path",
        "Avg Traj",
        "Avg Final Dist",
        "Avg ms",
    ]
    lines = ["| " + " | ".join(headers) + " |", "| " + " | ".join(["---"] * len(headers)) + " |"]
    for row in rows:
        lines.append(
            "| "
            + " | ".join(
                [
                    row["global_planner"],
                    f"{float(row['plan_success_rate']):.0%}",
                    f"{float(row['success_rate']):.0%}",
                    f"{float(row['collision_rate']):.0%}",
                    f"{float(row['avg_steps']):.1f}",
                    f"{float(row['avg_path_length']):.2f}",
                    f"{float(row['avg_traj_length']):.2f}",
                    f"{float(row['avg_final_distance']):.2f}",
                    f"{float(row['avg_elapsed_ms']):.2f}",
                ]
            )
            + " |"
        )
    return "\n".join(lines)


def _update_readme(readme: Path, table: str) -> None:
    content = readme.read_text()
    start = "<!-- BENCHMARK_TABLE_START -->"
    end = "<!-- BENCHMARK_TABLE_END -->"
    if start in content and end in content:
        before, rest = content.split(start, 1)
        _, after = rest.split(end, 1)
        updated = f"{before}{start}\n{table}\n{end}{after}"
    else:
        updated = (
            content
            + "\n## Benchmark Comparison\n\n"
            + start
            + "\n"
            + table
            + "\n"
            + end
            + "\n"
        )
    readme.write_text(updated)


def main() -> None:
    parser = argparse.ArgumentParser(description="Update README benchmark table.")
    parser.add_argument(
        "--summary-csv",
        type=Path,
        default=Path("reports/benchmark_summary.csv"),
    )
    parser.add_argument("--readme", type=Path, default=Path("README.md"))
    args = parser.parse_args()

    rows = _read_summary(args.summary_csv)
    if not rows:
        print("No summary rows found.")
        return
    table = _table(rows)
    _update_readme(args.readme, table)


if __name__ == "__main__":
    main()
