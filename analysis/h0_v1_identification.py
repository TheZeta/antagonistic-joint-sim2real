#!/usr/bin/env python3

import math
import sys

import numpy as np

from h0_quasistatic import (
    classify_branches,
    load_rows,
    settled_plateau,
    split_segments,
)


def state_sequence(offsets, transition_counts):
    z = 0.0
    states = [z]

    for previous, current in zip(
        offsets[:-1],
        offsets[1:],
    ):
        delta = current - previous

        if delta == 0.0:
            states.append(z)
            continue

        direction = 1.0 if delta > 0.0 else -1.0

        decay = math.exp(
            -abs(delta) / transition_counts
        )

        z = (
            direction
            + (z - direction) * decay
        )

        states.append(z)

    return np.asarray(states)


def rmse(actual, predicted):
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
            "Usage: h0_v1_identification.py "
            "<quasistatic-sweep.csv>"
        )
        return 1

    plateaus = []

    for segment in split_segments(
        load_rows(sys.argv[1])
    ):
        plateau = settled_plateau(segment)

        if plateau is not None:
            plateaus.append(plateau)

    classify_branches(plateaus)

    q_zero = plateaus[0]["q_deg"]

    offsets = np.asarray(
        [
            p["offset"]
            for p in plateaus
        ],
        dtype=float,
    )

    measured = np.asarray(
        [
            p["q_deg"] - q_zero
            for p in plateaus
        ],
        dtype=float,
    )

    best = None

    # Identify the displacement scale over which the
    # hysteresis state reverses direction.
    candidates = np.linspace(
        0.1,
        30.0,
        3000,
    )

    for transition_counts in candidates:
        z = state_sequence(
            offsets,
            transition_counts,
        )

        # For each candidate transition scale,
        # q = a*x + b + h*z is linear in a,b,h.
        design = np.column_stack(
            (
                offsets,
                np.ones_like(offsets),
                z,
            )
        )

        coefficients, _, _, _ = (
            np.linalg.lstsq(
                design,
                measured,
                rcond=None,
            )
        )

        prediction = (
            design @ coefficients
        )

        error = rmse(
            measured,
            prediction,
        )

        if (
            best is None
            or error < best["rmse"]
        ):
            best = {
                "transition_counts":
                    float(transition_counts),

                "slope":
                    float(coefficients[0]),

                "bias":
                    float(coefficients[1]),

                "memory_gain":
                    float(coefficients[2]),

                "rmse":
                    error,

                "states":
                    z,

                "prediction":
                    prediction,
            }

    print("H0 MODEL V1 IDENTIFICATION")
    print("=" * 45)

    print(
        "transition scale = "
        f"{best['transition_counts']:.3f} counts"
    )

    print(
        "slope = "
        f"{best['slope']:.4f} deg/count"
    )

    print(
        "bias = "
        f"{best['bias']:+.3f} deg"
    )

    print(
        "memory gain = "
        f"{best['memory_gain']:+.3f} deg"
    )

    print(
        "full memory branch separation = "
        f"{2.0 * abs(best['memory_gain']):.3f} deg"
    )

    print(
        "RMSE = "
        f"{best['rmse']:.3f} deg"
    )

    print()
    print("STATE TRACE")

    for plateau, z, predicted in zip(
        plateaus,
        best["states"],
        best["prediction"],
    ):
        measured_q = (
            plateau["q_deg"] - q_zero
        )

        print(
            f"x={plateau['offset']:+5.0f} "
            f"z={z:+.3f} "
            f"measured={measured_q:+.3f} "
            f"predicted={predicted:+.3f}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())