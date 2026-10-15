#!/usr/bin/env python3

import csv
import math
import statistics
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np


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


def branch_points(plateaus, branch):
    return sorted([(p["offset"], p["q_relative_deg"],) for p in plateaus if p["branch"] == branch], key=lambda point: point[0],)


def hysteresis_points(plateaus, outward_branch, return_branch,):
    outward = {
        abs(p["offset"]): p["q_relative_deg"]
        for p in plateaus
        if p["branch"] == outward_branch
    }

    returning = {
        abs(p["offset"]): p["q_relative_deg"]
        for p in plateaus
        if p["branch"] == return_branch
    }

    offsets = sorted(outward.keys() & returning.keys())

    return [(offset, abs(returning[offset] - outward[offset]),) for offset in offsets]


def plot_quasistatic_branches(plateaus, model_slope_deg_per_count, output_path,):
    fig, ax = plt.subplots(figsize=(8, 6))

    branch_specs = [
        ("positive_outward", "Positive outward", "o-",),
        ("positive_return", "Positive return", "s-",),
        ("negative_outward", "Negative outward", "o-",),
        ("negative_return", "Negative return", "s-",),
    ]

    for branch, label, style in branch_specs:
        points = branch_points(plateaus, branch,)

        xs = [point[0] for point in points]
        ys = [point[1] for point in points]

        ax.plot(xs, ys, style, label=label,)

    max_offset = max(abs(p["offset"]) for p in plateaus)
    model_x = [-max_offset, max_offset,]
    model_y = [model_slope_deg_per_count * x for x in model_x]

    ax.plot(model_x, model_y, "--", linewidth=2, label="Model v0",)
    ax.set_xlabel("Differential offset [counts]")
    ax.set_ylabel("Joint angle relative to initial preload [deg]")
    ax.set_title("H0 quasi-static response vs Model v0")
    ax.grid(True, alpha=0.3,)
    ax.legend()

    fig.tight_layout()
    fig.savefig(output_path, dpi=200,)

    plt.close(fig)


def plot_hysteresis(plateaus, output_path,):
    positive = hysteresis_points(plateaus, "positive_outward", "positive_return",)
    negative = hysteresis_points(plateaus, "negative_outward", "negative_return",)

    # Never silently produce a zero-hysteresis figure
    # from accidental duplicate branch selections.

    if not any(width > 0.1 for _, width in positive + negative):
        raise RuntimeError("Hysteresis plot unexpectedly collapsed to zero; check branch selection")

    fig, ax = plt.subplots(figsize=(8, 5))

    ax.plot([p[0] for p in positive], [p[1] for p in positive], "o-", label="Positive side",)
    ax.plot([p[0] for p in negative], [p[1] for p in negative], "s-", label="Negative side",)

    ax.set_xlabel("|Differential offset| [counts]")
    ax.set_ylabel("Hysteresis width [deg]")
    ax.set_title("H0 quasi-static hysteresis")
    ax.grid(True, alpha=0.3,)
    ax.legend()

    fig.tight_layout()
    fig.savefig(output_path, dpi=200)

    plt.close(fig)


def characterization_points(plateaus,):
    valid_branches = {"positive_outward", "positive_return", "negative_outward", "negative_return"}

    return [p for p in plateaus if p["branch"] in valid_branches]


def motion_direction(branch):
    # +1: differential offset increasing
    # -1: differential offset decreasing

    if branch in {"positive_outward", "negative_return",}:
        return 1.0

    if branch in {"positive_return", "negative_outward",}:
        return -1.0

    raise ValueError(f"No motion direction for {branch}")


def rmse(actual, predicted):
    error = actual - predicted

    return math.sqrt(float(np.mean(error ** 2)))


def compare_models(plateaus, nominal_slope,):
    points = characterization_points(plateaus)

    x = np.array([p["offset"] for p in points], dtype=float,)
    y = np.array([p["q_relative_deg"] for p in points], dtype=float,)
    direction = np.array([motion_direction(p["branch"]) for p in points], dtype=float,)

    # Model 0:
    # nominal geometry, no fitted parameters.
    model_0_prediction = nominal_slope * x
    model_0_rmse = rmse(y, model_0_prediction,)

    # Model 1:
    # one fitted gain, constrained through zero.
    fitted_slope = float(np.dot(x, y) / np.dot(x, x))

    model_1_prediction = fitted_slope * x
    model_1_rmse = rmse(y, model_1_prediction,)

    # Model 2:
    #
    # q = a*x + b + h*d
    #
    # d = +1 while differential target increases
    # d = -1 while it decreases
    design = np.column_stack((x, np.ones_like(x), direction,))
    coefficients, _, _, _ = np.linalg.lstsq(design, y, rcond=None,)
    slope = float(coefficients[0])
    bias = float(coefficients[1])
    direction_bias = float(coefficients[2])
    model_2_prediction = design @ coefficients
    model_2_rmse = rmse(y, model_2_prediction,)
    effective_radius_m = METERS_PER_COUNT / math.radians(fitted_slope)

    return {
        "model_0_rmse":
            model_0_rmse,

        "model_1_slope":
            fitted_slope,

        "model_1_rmse":
            model_1_rmse,

        "effective_radius_mm":
            effective_radius_m * 1000.0,

        "model_2_slope":
            slope,

        "model_2_bias":
            bias,

        "model_2_direction_bias":
            direction_bias,

        "model_2_hysteresis_width":
            2.0 * abs(direction_bias),

        "model_2_rmse":
            model_2_rmse,
    }


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

    branch_plot_path = (input_path.parent / "quasistatic_model_comparison.png")
    hysteresis_plot_path = (input_path.parent / "quasistatic_hysteresis.png")

    plot_quasistatic_branches(plateaus, model_slope_deg_per_count, branch_plot_path,)
    plot_hysteresis(plateaus, hysteresis_plot_path,)

    print()
    print("Figures written to:")
    print(branch_plot_path)
    print(hysteresis_plot_path)

    comparison = compare_models(plateaus, model_slope_deg_per_count,)

    print()
    print("MODEL COMPARISON")
    print("=" * 45)
    print("Model 0 - nominal geometry:")
    print(f"  RMSE = {comparison['model_0_rmse']:.3f} deg")
    print()
    print("Model 1 - fitted static gain:")
    print(f"  slope = {comparison['model_1_slope']:.4f} deg/count")
    print(f"  equivalent effective radius = {comparison['effective_radius_mm']:.2f} mm")
    print(f"  RMSE = {comparison['model_1_rmse']:.3f} deg")
    print()
    print("Model 2 - gain + direction memory:")
    print(f"  slope = {comparison['model_2_slope']:.4f} deg/count")
    print(f"  bias = {comparison['model_2_bias']:+.3f} deg")
    print(f"  direction bias = {comparison['model_2_direction_bias']:+.3f} deg")
    print(f"  implied branch separation = {comparison['model_2_hysteresis_width']:.3f} deg")
    print(f"  RMSE = {comparison['model_2_rmse']:.3f} deg")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
