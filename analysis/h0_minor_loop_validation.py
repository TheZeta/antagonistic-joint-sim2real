#!/usr/bin/env python3

import csv
import math
import sys

import numpy as np


OLD_SLOPE = 0.2782
OLD_BIAS = -0.297
OLD_MEMORY_GAIN = -0.878

V1_SLOPE = 0.2821
V1_BIAS = -0.284
V1_MEMORY_GAIN = -0.956
V1_TRANSITION_COUNTS = 3.310


def load_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


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


def settled_q(segment):
    end = float(segment[-1]["time_s"])

    values = [
        float(row["q_deg"])
        for row in segment
        if (
            int(row["state"]) == 3
            and float(row["time_s"])
            >= end - 2.0
        )
    ]

    if not values:
        return None

    return float(np.mean(values))


def rmse(actual, predicted):
    return math.sqrt(
        float(
            np.mean(
                (
                    np.asarray(actual)
                    - np.asarray(predicted)
                ) ** 2
            )
        )
    )


def update_memory(z, old_x, new_x):
    delta = new_x - old_x

    if delta == 0:
        return z

    direction = 1.0 if delta > 0 else -1.0

    decay = math.exp(
        -abs(delta)
        / V1_TRANSITION_COUNTS
    )

    return (
        direction
        + (z - direction) * decay
    )


def main():
    if len(sys.argv) != 2:
        print(
            "Usage: h0_minor_loop_validation.py "
            "<minor-loop.csv>"
        )
        return 1

    plateaus = []

    for segment in split_segments(
        load_rows(sys.argv[1])
    ):
        m1, m2 = target_pair(segment[0])

        if (m1 + m2) / 2 != 100:
            continue

        q = settled_q(segment)

        if q is None:
            continue

        plateaus.append({
            "offset": (m2 - m1) / 2,
            "q_deg": q,
        })

    if not plateaus:
        raise RuntimeError(
            "No characterization plateaus found"
        )

    # First long (100,100) measurement.
    initial = plateaus[0]
    q_zero = initial["q_deg"]

    z = (
        0.0 - V1_BIAS
    ) / V1_MEMORY_GAIN

    z = float(
        np.clip(z, -1.0, 1.0)
    )

    previous_offset = 0.0

    measured = []
    old_prediction = []
    v1_prediction = []

    records = []

    for point in plateaus[1:]:
        x = point["offset"]

        direction = (
            1.0
            if x > previous_offset
            else -1.0
        )

        old_q = (
            OLD_SLOPE * x
            + OLD_BIAS
            + OLD_MEMORY_GAIN * direction
        )

        z = update_memory(
            z,
            previous_offset,
            x,
        )

        v1_q = (
            V1_SLOPE * x
            + V1_BIAS
            + V1_MEMORY_GAIN * z
        )

        measured_q = (
            point["q_deg"]
            - q_zero
        )

        measured.append(measured_q)
        old_prediction.append(old_q)
        v1_prediction.append(v1_q)

        records.append(
            (
                x,
                z,
                measured_q,
                old_q,
                v1_q,
            )
        )

        previous_offset = x

    print("H0 MINOR-LOOP VALIDATION")
    print("=" * 45)

    print(
        "Direction-bit RMSE: "
        f"{rmse(measured, old_prediction):.3f} deg"
    )

    print(
        "Model v1 RMSE: "
        f"{rmse(measured, v1_prediction):.3f} deg"
    )

    print()
    print("PARTIAL REVERSALS")

    for x, z, measured_q, old_q, v1_q in records:
        if abs(x) == 18:
            print(
                f"x={x:+.0f} "
                f"z={z:+.3f} "
                f"measured={measured_q:+.3f} "
                f"direction-bit={old_q:+.3f} "
                f"v1={v1_q:+.3f}"
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
