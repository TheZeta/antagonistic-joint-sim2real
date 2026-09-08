#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


RESULTS_DIR = Path("results")

P_PATH = RESULTS_DIR / "p_control_response.csv"
PD_PATH = RESULTS_DIR / "pd_control_response.csv"
PD_FF_PATH = RESULTS_DIR / "pd_feedforward_response.csv"

OUTPUT_PATH = RESULTS_DIR / "control_comparison.png"


def load_response(path):
    time = []
    reference = []
    position = []

    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)

        for row in reader:
            time.append(float(row["time_s"]))
            reference.append(float(row["q_reference_rad"]))
            position.append(float(row["q_rad"]))

    return time, reference, position


def radians_to_degrees(values):
    return [value * 180.0 / math.pi for value in values]


def main():
    p_time, reference, p_position = load_response(P_PATH)
    pd_time, _, pd_position = load_response(PD_PATH)
    ff_time, _, ff_position = load_response(PD_FF_PATH)

    reference_deg = radians_to_degrees(reference)
    p_deg = radians_to_degrees(p_position)
    pd_deg = radians_to_degrees(pd_position)
    ff_deg = radians_to_degrees(ff_position)

    plt.figure()

    plt.plot(
        p_time,
        p_deg,
        label="P"
    )

    plt.plot(
        pd_time,
        pd_deg,
        label="PD"
    )

    plt.plot(
        ff_time,
        ff_deg,
        label="PD + feedforward"
    )

    plt.plot(
        p_time,
        reference_deg,
        linestyle="--",
        label="Reference"
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Joint angle [deg]")
    plt.title("Closed-loop position-control comparison")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        OUTPUT_PATH,
        dpi=200
    )

    plt.close()

    print(f"Plot written to {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
