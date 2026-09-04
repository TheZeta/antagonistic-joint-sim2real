#!/usr/bin/env python3

import csv
from pathlib import Path

import matplotlib.pyplot as plt


RESULTS_DIR = Path("results")
CSV_PATH = RESULTS_DIR / "open_loop_validation.csv"


def load_data():
    time = []
    q_rk4 = []
    q_analytical = []
    q_error = []

    with CSV_PATH.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)

        for row in reader:
            time.append(float(row["time_s"]))
            q_rk4.append(float(row["q_rk4_rad"]))
            q_analytical.append(float(row["q_analytical_rad"]))
            q_error.append(float(row["q_error_rad"]))

        return time, q_rk4, q_analytical, q_error


def plot_position(time, q_rk4, q_analytical):
    plt.figure()

    plt.plot(
        time,
        q_analytical,
        label="Analytical"
    )

    plt.plot(
        time,
        q_rk4,
        linestyle="--",
        label="RK4"
    )

    plt.axhline(
        0.25,
        linestyle=":",
        label="Equilibrium"
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Joint angle [rad]")
    plt.title("Open-loop step response validation")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        RESULTS_DIR / "open_loop_validation_position.png",
        dpi=200
    )

    plt.close()


def plot_error(time, q_error):
    plt.figure()

    plt.plot(time, q_error)

    plt.xlabel("Time [s]")
    plt.ylabel("Position error [rad]")
    plt.title("RK4 position error relative to analytical solution")
    plt.grid(True)
    plt.tight_layout()

    plt.savefig(
        RESULTS_DIR / "open_loop_validation_error.png",
        dpi=200
    )

    plt.close()


def main():
    time, q_rk4, q_analytical, q_error = load_data()

    plot_position(
        time,
        q_rk4,
        q_analytical
    )

    plot_error(
        time,
        q_error
    )

    print("Plots written to results/")


if __name__ == "__main__":
    main()
