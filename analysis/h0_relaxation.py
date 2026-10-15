#!/usr/bin/env python3

import csv
import math
import statistics
import sys

import numpy as np


def load_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def trim_stale_prefix(rows):
    # The relaxation capture contains a few stale telemetry
    # rows before the new run begins. Keep the long monotonic
    # device-time run after the first backwards jump.
    for i in range(1, len(rows)):
        previous = float(rows[i - 1]["time_s"])
        current = float(rows[i]["time_s"])

        if current < previous:
            return rows[i:]

    return rows


def target_pair(row):
    return(int(row["m1_target_counts"]), int(row["m2_target_counts"]),)


def split_segments(rows):
    segments = []
    current = []

    for row in rows:
        if (current and target_pair(row) != target_pair(current[-1])):
            segments.append(current)
            current = []

        current.append(row)

    if current:
        segments.append(current)

    return segments


def fit_relaxation(segment):
    holding = [row for row in segment if int(row["state"]) == 3]

    if len(holding) < 50:
        return None

    t = np.array([float(row["time_s"]) for row in holding], dtype=float,)
    q = np.array([float(row["q_deg"]) for row in holding], dtype=float,)

    t -= t[0]
    mask = t <= 10.0
    t = t[mask]
    q = q[mask]

    if len(t) < 20:
        return None

    taus = np.linspace(0.05, 3.0, 1500,)
    best = None
    for tau in taus:
        decay = np.exp(-t / tau)
        design = np.column_stack((np.ones_like(t), decay,))
        coefficients, _, _, _ = (np.linalg.lstsq(design, q, rcond=None,))
        prediction = (design @ coefficients)
        rmse = math.sqrt(float(np.mean((q - prediction) ** 2)))
        if best is None or rmse < best["rmse"]:
            best = {
                "tau": float(tau),
                "q_inf": float(coefficients[0]),
                "amplitude": float(coefficients[1]),
                "rmse": rmse,
            }

    return best


def main():
    if len(sys.argv) != 2:
        print("Usage: h0_relaxation.py <relaxation-probe.csv>")

        return 1

    rows = trim_stale_prefix(load_rows(sys.argv[1]))
    fits = []
    for segment in split_segments(rows):
        m1, m2 = target_pair(segment[0])

        common = 0.5 * (m1 + m2)
        offset = 0.5 * (m2 - m1)

        # Only the deliberate long-dwell characterization
        # states around the +100-count preload center.
        if common != 100:
            continue

        if offset not in { -20, -10, 0, 10, 20 }:
            continue

        fit = fit_relaxation(segment)
        if fit is None:
            continue

        # Ignore essentially stationary initial plateaus.
        if abs(fit["amplitude"]) < 0.25:
            continue

        fit["m1"] = m1
        fit["m2"] = m2
        fit["offset"] = offset

        fits.append(fit)

    if not fits:
        raise RuntimeError("No relaxation transitions found")

    print("H0 RELAXATION IDENTIFICATION")
    print("=" * 45)

    for fit in fits:
        print(
            f"({fit['m1']:3d},"
            f"{fit['m1']:3d}),"
            f"offset={fit['m1']:3d},"
            f"tau={fit['tau']:.3f},"
            f"RMSE={fit['rmse']:.3f},"
        )

    taus = [fit["tau"] for fit in fits]

    print()
    print(f"Median tau: {statistics.median(taus):.3f} s")
    print(f"Range: {min(taus):.3f} to {max(taus):.3f} s")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
