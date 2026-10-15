#!/usr/bin/env python3

import csv
import math
import statistics
import sys
from pathlib import Path

METERS_PER_COUNT = 0.023 / 201.0
JOINT_RADIUS_M = 0.020

MIN_PLATEAU_S = 4.0
SETTLED_WINDOW_S = 2.0


def mean(values):
    return statistics.fmean(values)


def linear_fit(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    xbar = mean(xs)
    ybar = mean(ys)

    sxx = sum((x - xbar) ** 2 for x in xs)
    sxy = sum(
        (x - xbar) * (y - ybar)
        for x, y in points
    )

    slope = sxy / sxx
    intercept = ybar - slope * xbar

    ss_res = sum(
        (y - (slope * x + intercept)) ** 2
        for x, y in points
    )

    ss_tot = sum(
        (y - ybar) ** 2
        for y in ys
    )

    r2 = 1.0 - ss_res / ss_tot

    return slope, intercept, r2


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


def settled_plateau(segment):
    start = float(segment[0]["time_s"])
    end = float(segment[-1]["time_s"])

    if end - start < MIN_PLATEAU_S:
        return None

    tail_start = end - SETTLED_WINDOW_S

    tail = [
        row for row in segment
        if float(row["time_s"]) >= tail_start
    ]

    if not tail:
        return None

    # HOLDING = 3
    if any(int(row["state"]) != 3 for row in tail):
        return None

    m1_target, m2_target = target_pair(segment[0])

    offset = (
        m2_target - m1_target
    ) / 2.0

    common_target = (
        m1_target + m2_target
    ) / 2.0

    q_values = [
        float(row["q_deg"])
        for row in tail
    ]

    m1_errors = [
        int(row["m1_winding_counts"])
        - m1_target
        for row in tail
    ]

    m2_errors = [
        int(row["m2_winding_counts"])
        - m2_target
        for row in tail
    ]

    return {
        "m1_target": m1_target,
        "m2_target": m2_target,
        "offset": offset,
        "common_target": common_target,
        "q_deg": mean(q_values),
        "q_std_deg":
            statistics.pstdev(q_values),
        "m1_error_counts": mean(m1_errors),
        "m2_error_counts": mean(m2_errors),
        "common_winding_m": mean([
            float(row["common_winding_m"])
            for row in tail
        ]),
        "differential_winding_m": mean([
            float(row["differential_winding_m"])
            for row in tail
        ]),
        "faults": sum(
            int(row["fault"]) != 0
            for row in tail
        ),
    }


def classify_branches(plateaus):
    previous_offset = None
    zero_count = 0

    for p in plateaus:
        offset = p["offset"]

        if offset == 0:
            zero_count += 1

            if zero_count == 1:
                branch = "center_initial"
            elif previous_offset > 0:
                branch = "center_after_positive"
            else:
                branch = "center_after_negative"

        elif offset > 0:
            if (
                previous_offset is None
                or offset > previous_offset
            ):
                branch = "positive_outward"
            else:
                branch = "positive_return"

        else:
            if (
                previous_offset is None
                or offset < previous_offset
            ):
                branch = "negative_outward"
            else:
                branch = "negative_return"

        p["branch"] = branch
        previous_offset = offset


def main():
    if len(sys.argv) != 2:
        print(
            "Usage: h0_quasistatic.py "
            "<quasistatic-sweep.csv>"
        )
        return 1

    input_path = Path(sys.argv[1])

    rows = load_rows(input_path)

    plateaus = []

    for segment in split_segments(rows):
        plateau = settled_plateau(segment)

        if plateau is not None:
            plateaus.append(plateau)

    if not plateaus:
        raise RuntimeError(
            "No characterization plateaus found"
        )

    classify_branches(plateaus)

    initial = next(
        p for p in plateaus
        if p["branch"] == "center_initial"
    )

    q_zero = initial["q_deg"]

    model_slope_deg_per_count = math.degrees(
        METERS_PER_COUNT
        / JOINT_RADIUS_M
    )

    for p in plateaus:
        p["q_relative_deg"] = (
            p["q_deg"] - q_zero
        )

        p["model_q_deg"] = (
            p["offset"]
            * model_slope_deg_per_count
        )

        p["model_error_deg"] = (
            p["q_relative_deg"]
            - p["model_q_deg"]
        )

    output_path = (
        input_path.parent
        / "quasistatic_plateaus.csv"
    )

    fields = [
        "branch",
        "offset",
        "m1_target",
        "m2_target",
        "q_relative_deg",
        "q_std_deg",
        "model_q_deg",
        "model_error_deg",
        "m1_error_counts",
        "m2_error_counts",
        "common_winding_m",
        "differential_winding_m",
    ]

    with open(
        output_path,
        "w",
        newline="",
    ) as f:
        writer = csv.DictWriter(
            f,
            fieldnames=fields,
        )

        writer.writeheader()

        for p in plateaus:
            writer.writerow({
                key: p[key]
                for key in fields
            })

    print()
    print("H0 QUASI-STATIC CHARACTERIZATION")
    print("=" * 45)

    print(
        f"Plateaus detected: {len(plateaus)}"
    )

    print(
        "Model-v0 slope: "
        f"{model_slope_deg_per_count:.4f} "
        "deg/count"
    )

    print()
    print("BRANCH FITS")

    branch_names = [
        "positive_outward",
        "positive_return",
        "negative_outward",
        "negative_return",
    ]

    for name in branch_names:
        points = [
            (
                p["offset"],
                p["q_relative_deg"],
            )
            for p in plateaus
            if p["branch"] == name
        ]

        slope, intercept, r2 = linear_fit(
            points
        )

        print(
            f"{name:18s} "
            f"slope={slope:+.4f} "
            f"intercept={intercept:+.3f} "
            f"R2={r2:.5f}"
        )

    print()
    print("HYSTERESIS")

    for sign, outward, returning in [
        (
            "+",
            "positive_outward",
            "positive_return",
        ),
        (
            "-",
            "negative_outward",
            "negative_return",
        ),
    ]:
        outward_points = {
            abs(p["offset"]): p
            for p in plateaus
            if p["branch"] == outward
        }

        return_points = {
            abs(p["offset"]): p
            for p in plateaus
            if p["branch"] == returning
        }

        for offset in sorted(
            outward_points.keys()
            & return_points.keys()
        ):
            q1 = outward_points[offset][
                "q_relative_deg"
            ]

            q2 = return_points[offset][
                "q_relative_deg"
            ]

            width = abs(q2 - q1)

            print(
                f"{sign}{offset:4.0f} counts: "
                f"{width:.3f} deg"
            )

    print()
    print(
        "Plateau table written to:"
    )
    print(output_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())