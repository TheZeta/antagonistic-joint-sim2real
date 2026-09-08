#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


CSV_PATH = Path("results/model_mismatch_sweep.csv")
OUTPUT_PATH = Path("results/model_mismatch_sweep.png")


def to_degrees(value):
    return value * 180.0 / math.pi


def main():
    time = []
    reference = []
    q_low = []
    q_nominal = []
    q_high = []

    with CSV_PATH.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)

        for row in reader:
            time.append(float(row["time_s"]))
            reference.append(to_degrees(float(row["q_reference_rad"])))
            q_low.append(to_degrees(float(row["q_rj_0p018_rad"])))
            q_nominal.append(to_degrees(float(row["q_rj_0p020_rad"])))
            q_high.append(to_degrees(float(row["q_rj_0p022_rad"])))

    plt.figure()

    plt.plot(
        time,
        q_low,
        label="True r_j = 0.018 m"
    )

    plt.plot(
        time,
        q_nominal,
        label="True r_j = 0.020 m"
    )

    plt.plot(
        time,
        q_high,
        label="True r_j = 0.022 m"
    )

    plt.plot(
        time,
        reference,
        linestyle="--",
        label="Reference"
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Joint angle [deg]")
    plt.title("Effect of plant-model mismatch on position tracking")
    plt.grid("True")
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
