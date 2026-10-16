#!/usr/bin/env python3

import csv
import math
import sys

import numpy as np


# Old direction-bit model.
OLD_SLOPE = 0.2782
OLD_BIAS = -0.297
OLD_DIRECTION_BIAS = -0.878

# Model v1.
V1_SLOPE = 0.2821
V1_BIAS = -0.284
V1_MEMORY_GAIN = -0.956
V1_TRANSITION_COUNTS = 3.310

# Identified relaxation.
TAU_S = 0.59

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

    return (
        0.5 * (m1 + m2),
        0.5 * (m2 - m1),
    )


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
            "No settled samples found"
        )

    return float(np.mean(values))


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


def old_equilibrium(offset, direction):
    return (
        OLD_SLOPE * offset
        + OLD_BIAS
        + OLD_DIRECTION_BIAS * direction
    )


def v1_equilibrium(offset, memory_state):
    return (
        V1_SLOPE * offset
        + V1_BIAS
        + V1_MEMORY_GAIN * memory_state
    )


def update_memory(
    memory_state,
    previous_offset,
    new_offset,
):
    delta = new_offset - previous_offset

    if delta == 0.0:
        return memory_state

    direction = (
        1.0 if delta > 0.0 else -1.0
    )

    decay = math.exp(
        -abs(delta)
        / V1_TRANSITION_COUNTS
    )

    return (
        direction
        + (memory_state - direction)
        * decay
    )


def relaxation_step(
    q,
    equilibrium,
    dt_s,
):
    if dt_s <= 0.0:
        return q

    decay = math.exp(
        -dt_s / TAU_S
    )

    return (
        equilibrium
        + (q - equilibrium) * decay
    )


def main():
    if len(sys.argv) != 2:
        print(
            "Usage: h0_model_v1_validation.py "
            "<relaxation-probe.csv>"
        )
        return 1

    rows = trim_stale_prefix(
        load_rows(sys.argv[1])
    )

    segments = split_segments(rows)

    initial_index = None

    # Locate the long settled (100,100) state
    # immediately before the actual relaxation probe.
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
            and segment_duration(segment) >= 10.0
            and next_common == PRELOAD_COUNTS
            and next_offset != 0.0
        ):
            initial_index = i
            break

    if initial_index is None:
        raise RuntimeError(
            "Could not locate initial preload"
        )

    initial_segment = segments[
        initial_index
    ]

    # Experimental angle zero.
    q_zero = tail_mean_q(
        initial_segment
    )

    previous_offset = 0.0
    previous_direction = 1.0

    # Both empirical dynamic models begin at
    # the measured initial state.
    q_old = 0.0
    q_v1 = 0.0

    # Infer Model-v1 internal memory from the
    # measured initial equilibrium q=0 at x=0.
    memory_state = (
        0.0 - V1_BIAS
    ) / V1_MEMORY_GAIN

    memory_state = float(
        np.clip(
            memory_state,
            -1.0,
            1.0,
        )
    )

    initial_memory_state = memory_state

    previous_time = float(
        initial_segment[-1]["time_s"]
    )

    measured = []
    old_prediction = []
    v1_prediction = []

    elapsed = []
    offsets = []
    memory_trace = []
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

        # Direction-bit model.
        if offset > previous_offset:
            direction = 1.0
        elif offset < previous_offset:
            direction = -1.0
        else:
            direction = previous_direction

        old_eq = old_equilibrium(
            offset,
            direction,
        )

        # Model v1 state changes only when
        # differential displacement changes.
        memory_state = update_memory(
            memory_state,
            previous_offset,
            offset,
        )

        v1_eq = v1_equilibrium(
            offset,
            memory_state,
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

            q_old = relaxation_step(
                q_old,
                old_eq,
                dt_s,
            )

            q_v1 = relaxation_step(
                q_v1,
                v1_eq,
                dt_s,
            )

            q_measured = (
                float(row["q_deg"])
                - q_zero
            )

            measured.append(q_measured)

            old_prediction.append(q_old)
            v1_prediction.append(q_v1)

            elapsed.append(
                time_s - start_time
            )

            offsets.append(offset)
            memory_trace.append(
                memory_state
            )

            states.append(
                int(row["state"])
            )

            previous_time = time_s

        previous_offset = offset
        previous_direction = direction

    measured = np.asarray(measured)

    old_prediction = np.asarray(
        old_prediction
    )

    v1_prediction = np.asarray(
        v1_prediction
    )

    states = np.asarray(states)

    holding = states == 3

    old_rmse = rmse(
        measured,
        old_prediction,
    )

    v1_rmse = rmse(
        measured,
        v1_prediction,
    )

    old_holding_rmse = rmse(
        measured[holding],
        old_prediction[holding],
    )

    v1_holding_rmse = rmse(
        measured[holding],
        v1_prediction[holding],
    )

    improvement = (
        100.0
        * (old_rmse - v1_rmse)
        / old_rmse
    )

    print(
        "H0 MODEL V1 VALIDATION"
    )

    print("=" * 45)

    print(
        "Initial inferred memory state: "
        f"{initial_memory_state:+.3f}"
    )

    print()

    print(
        "Direction-bit dynamic RMSE: "
        f"{old_rmse:.3f} deg"
    )

    print(
        "Model v1 dynamic RMSE: "
        f"{v1_rmse:.3f} deg"
    )

    print(
        "Direction-bit holding RMSE: "
        f"{old_holding_rmse:.3f} deg"
    )

    print(
        "Model v1 holding RMSE: "
        f"{v1_holding_rmse:.3f} deg"
    )

    print()

    print(
        "Model v1 improvement over "
        "direction-bit model: "
        f"{improvement:.1f}%"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
