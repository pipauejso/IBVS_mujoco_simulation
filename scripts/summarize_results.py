"""Aggregate the five measured experiments without rerunning simulation."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1] / "results"
SCENARIOS = ["translation", "depth", "rotation", "tilt", "combined"]


def main():
    summaries = [json.loads((ROOT / scenario / "summary.json").read_text(encoding="utf-8"))
                 for scenario in SCENARIOS]
    with (ROOT / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)

    fig, ax = plt.subplots(figsize=(9, 5))
    for scenario in SCENARIOS:
        with (ROOT / scenario / "trajectory.csv").open(encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        times = [float(row["time_s"]) for row in rows]
        errors = [float(row["error_rms_px"]) if row["error_rms_px"] else np.nan for row in rows]
        ax.plot(times, errors, label=scenario)
    ax.axhline(1., color="gray", linestyle="--", linewidth=1, label="Acceptance: 1 px")
    ax.set(xlabel="Simulation time (s)", ylabel="Feature RMS (px)",
           title="UR5e IBVS: five rendered-image experiments")
    ax.grid(alpha=.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(ROOT / "comparison.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(5, 3, figsize=(10, 12))
    for row, (scenario, summary) in enumerate(zip(SCENARIOS, summaries)):
        for column, name in enumerate(["reference", "initial", "final"]):
            axes[row, column].imshow(plt.imread(ROOT / scenario / f"{name}.png"))
            axes[row, column].set_xticks([])
            axes[row, column].set_yticks([])
            if row == 0:
                axes[row, column].set_title(name.capitalize())
            if column == 0:
                axes[row, column].set_ylabel(scenario)
            if column == 2:
                axes[row, column].set_xlabel(f"RMS = {summary['final_error_rms_px']:.3f} px")
    fig.suptitle("UR5e IBVS: actual camera renders (green: detected, yellow: desired)")
    fig.tight_layout()
    fig.savefig(ROOT / "overview.png", dpi=150)
    plt.close(fig)
    print(f"Saved summary.csv, comparison.png, overview.png to {ROOT}")
    return 0 if all(item["passed"] for item in summaries) else 1


if __name__ == "__main__":
    raise SystemExit(main())
