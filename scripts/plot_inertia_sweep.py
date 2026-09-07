#!/usr/bin/env python3

import csv
from pathlib import Path

import matplotlib.pyplot as plt


CSV_PATH = Path("results/inertia_sweep.csv")
OUTPUT_PATH = Path("results/inertia_sweep.png")


def main():
    time = []
    q_low = []
    q_nominal = []
    q_high = []

    with CSV_PATH.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)

        for row in reader:
            time.append(float(row["time_s"]))
            q_low.append(float(row["q_I_0p005_rad"]))
            q_nominal.append(float(row["q_I_0p010_rad"]))
            q_high.append(float(row["q_I_0p020_rad"]))

    plt.figure()

    plt.plot(
        time,
        q_low,
        label="I = 0.005 kg m²"
    )

    plt.plot(
        time,
        q_nominal,
        label="I = 0.010 kg m²"
    )

    plt.plot(
        time,
        q_high,
        label="I = 0.020 kg m²"
    )

    plt.axhline(
        0.25,
        linestyle="--",
        label="Equilibrium"
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Joint angle [rad]")
    plt.title("Effect of link inertia on step response")
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
