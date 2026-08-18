#!/usr/bin/env python3
# Copyright (c) 2026 Amal Dev Haridevan
# SPDX-License-Identifier: MIT

"""Plot the state, reference, and input data in simulation_log.csv."""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


STATE_GROUPS = (
    ("Position", ("x", "y", "z")),
    ("Velocity", ("vx", "vy", "vz")),
    ("Quaternion", ("qw", "qx", "qy", "qz")),
    ("Angular velocity", ("wx", "wy", "wz")),
)
CONTROL_NAMES = ("u0", "u1", "u2", "u3")


def read_log(path: Path) -> dict[str, list[float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {
            "time",
            *CONTROL_NAMES,
            *(name for _, names in STATE_GROUPS for name in names),
            *(f"{name}_des" for _, names in STATE_GROUPS for name in names),
        }
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(f"{path} is missing columns: {', '.join(sorted(missing))}")

        columns = {name: [] for name in required}
        for row in reader:
            for name in columns:
                columns[name].append(float(row[name]))

    if not columns["time"]:
        raise ValueError(f"{path} contains no simulation samples")
    return columns


def plot_log(columns: dict[str, list[float]]):
    times = columns["time"]
    figure, axes = plt.subplots(5, 1, figsize=(11, 15), sharex=True)

    for axis, (title, names) in zip(axes, STATE_GROUPS):
        for name in names:
            line = axis.plot(times, columns[name], label=name)[0]
            axis.plot(
                times,
                columns[f"{name}_des"],
                "--",
                color=line.get_color(),
                label=f"{name}_des",
            )
        axis.set_ylabel(title)
        axis.grid(True, alpha=0.3)
        axis.legend(ncol=min(4, len(names)), fontsize="small")

    for name in CONTROL_NAMES:
        axes[-1].plot(times, columns[name], label=name)
    axes[-1].set_ylabel("Control")
    axes[-1].set_xlabel("Time [s]")
    axes[-1].grid(True, alpha=0.3)
    axes[-1].legend(ncol=4, fontsize="small")

    figure.suptitle("Quadrotor closed-loop simulation")
    figure.tight_layout()
    return figure


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "log",
        nargs="?",
        type=Path,
        default=Path("simulation_log.csv"),
        help="CSV log to plot (default: simulation_log.csv)",
    )
    parser.add_argument("--save", type=Path, help="Save the figure to this path")
    parser.add_argument(
        "--no-show", action="store_true", help="Do not open an interactive window"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    figure = plot_log(read_log(args.log))
    if args.save:
        figure.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"Saved plot to {args.save}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
