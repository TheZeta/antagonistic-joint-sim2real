#!/usr/bin/env python3

import csv
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


SLOPE_DEG_PER_COUNT = 0.2782
BIAS_DEG = -0.297
DIRECTION_BIAS_DEG = -0.878
RELAXATION_TIME_S = 0.59

PRELOAD_COUNTS = 100


def load_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def trim_stale_prefix(rows):
    for i in range(1, len(rows)):
        previous = float(rows[i - 1]["time_s"])
        current = float(rows[i]["time_s"])

        if current < previous:
            return rows[i:]

    return rows


def target_pair(row):
    return (
        int(row["m1_target_counts"]),
        int(row["m2_target_counts"]),
    )


def split_segments(rows):
    segments = []
    current = []

    for row in rows:
        if (
            current
            and target_pair(row)
            != target_pair(current[-1])
        ):
            segments.append(current)
            current = []

        current.append(row)

    if current:
        segments.append(current)

    return segments


def segment_duration(segment):
    return (
        float(segment[-1]["time_s"])
        - float(segment[0]["time_s"])
    )


def common_and_offset(segment):
    m1, m2 = target_pair(segment[0])

    common = 0.5 * (m1 + m2)
    offset = 0.5 * (m2 - m1)

    return common, offset


def tail_mean_q(segment, window_s=2.0):
    end_time = float(segment[-1]["time_s"])

    values = [
        float(row["q_deg"])
        for row in segment
        if (
            int(row["state"]) == 3
            and float(row["time_s"])
                >= end_time - window_s
        )
    ]

    if not values:
        raise RuntimeError(
            "Could not find settled preload samples"
        )

    return float(np.mean(values))


def equilibrium_deg(offset, direction):
    return (
        SLOPE_DEG_PER_COUNT * offset
        + BIAS_DEG
        + DIRECTION_BIAS_DEG * direction
    )


def rmse(actual, predicted):
    actual = np.asarray(actual)
    predicted = np.asarray(predicted)

    return math.sqrt(
        float(
            np.mean(
                (actual - predicted) ** 2
            )
        )
    )


def main():
    if len(sys.argv) != 2:
        print(
            "Usage: h0_dynamic_validation.py "
            "<relaxation-probe.csv>"
        )
        return 1

    input_path = Path(sys.argv[1])

    rows = trim_stale_prefix(
        load_rows(input_path)
    )

    segments = split_segments(rows)

    initial_index = None

    for i in range(len(segments) - 1):
        segment = segments[i]

        m1, m2 = target_pair(segment[0])

        next_common, next_offset = (
            common_and_offset(
                segments[i + 1]
            )
        )

        if (
            m1 == PRELOAD_COUNTS
            and m2 == PRELOAD_COUNTS
            and segment_duration(segment)
                >= 10.0
            and next_common
                == PRELOAD_COUNTS
            and next_offset != 0
        ):
            initial_index = i
            break

    if initial_index is None:
        raise RuntimeError(
            "Could not locate initial preload state"
        )

    initial_segment = segments[
        initial_index
    ]

    q_zero_deg = tail_mean_q(
        initial_segment
    )

    previous_time = float(
        initial_segment[-1]["time_s"]
    )

    previous_offset = 0.0
    previous_direction = 1.0

    q_model_deg = 0.0

    measured = []
    predicted = []
    equilibrium = []

    elapsed = []
    offsets = []
    states = []

    start_time = None

    for segment in segments[
        initial_index + 1:
    ]:
        common, offset = (
            common_and_offset(segment)
        )

        if common != PRELOAD_COUNTS:
            break

        if offset > previous_offset:
            direction = 1.0
        elif offset < previous_offset:
            direction = -1.0
        else:
            direction = previous_direction

        q_equilibrium_deg = (
            equilibrium_deg(
                offset,
                direction,
            )
        )

        for row in segment:
            time_s = float(
                row["time_s"]
            )

            if start_time is None:
                start_time = time_s

            dt_s = max(
                0.0,
                time_s - previous_time,
            )

            decay = math.exp(
                -dt_s
                / RELAXATION_TIME_S
            )

            q_model_deg = (
                q_equilibrium_deg
                + (
                    q_model_deg
                    - q_equilibrium_deg
                )
                * decay
            )

            q_measured_deg = (
                float(row["q_deg"])
                - q_zero_deg
            )

            measured.append(
                q_measured_deg
            )

            predicted.append(
                q_model_deg
            )

            equilibrium.append(
                q_equilibrium_deg
            )

            elapsed.append(
                time_s - start_time
            )

            offsets.append(offset)

            states.append(
                int(row["state"])
            )

            previous_time = time_s

        previous_offset = offset
        previous_direction = direction

    measured = np.asarray(measured)
    predicted = np.asarray(predicted)
    equilibrium = np.asarray(equilibrium)
    states = np.asarray(states)

    holding = states == 3

    static_rmse = rmse(
        measured,
        equilibrium,
    )

    dynamic_rmse = rmse(
        measured,
        predicted,
    )

    holding_dynamic_rmse = rmse(
        measured[holding],
        predicted[holding],
    )

    improvement = (
        100.0
        * (
            static_rmse
            - dynamic_rmse
        )
        / static_rmse
    )

    print(
        "H0 DYNAMIC MODEL VALIDATION"
    )

    print("=" * 45)

    print(
        "Memoryless branch RMSE: "
        f"{static_rmse:.3f} deg"
    )

    print(
        "Dynamic model RMSE: "
        f"{dynamic_rmse:.3f} deg"
    )

    print(
        "Holding-only dynamic RMSE: "
        f"{holding_dynamic_rmse:.3f} deg"
    )

    print(
        "Dynamic improvement: "
        f"{improvement:.1f}%"
    )

    output_csv = (
        input_path.parent
        / "dynamic_model_validation.csv"
    )

    with open(
        output_csv,
        "w",
        newline="",
    ) as f:
        writer = csv.writer(f)

        writer.writerow([
            "elapsed_s",
            "offset_counts",
            "q_measured_deg",
            "q_predicted_deg",
            "q_equilibrium_deg",
            "state",
        ])

        for values in zip(
            elapsed,
            offsets,
            measured,
            predicted,
            equilibrium,
            states,
        ):
            writer.writerow(values)

    plot_path = (
        input_path.parent
        / "dynamic_model_validation.png"
    )

    fig, ax = plt.subplots(
        figsize=(10, 5)
    )

    ax.plot(
        elapsed,
        measured,
        label="Measured H0",
    )

    ax.plot(
        elapsed,
        predicted,
        label="Dynamic model",
    )

    ax.plot(
        elapsed,
        equilibrium,
        "--",
        label="Quasi-static equilibrium",
    )

    ax.set_xlabel(
        "Elapsed time [s]"
    )

    ax.set_ylabel(
        "Joint angle relative to initial preload [deg]"
    )

    ax.set_title(
        "H0 relaxation: measured vs empirical model"
    )

    ax.grid(
        True,
        alpha=0.3,
    )

    ax.legend()

    fig.tight_layout()

    fig.savefig(
        plot_path,
        dpi=200,
    )

    plt.close(fig)

    print()
    print("Outputs:")
    print(output_csv)
    print(plot_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
