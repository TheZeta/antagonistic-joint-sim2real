#!/usr/bin/env python3

import csv
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from h0_quasistatic import (
    load_rows,
    settled_plateau,
    split_segments,
)


def rmse(actual, predicted):
    actual = np.asarray(
        actual,
        dtype=float,
    )

    predicted = np.asarray(
        predicted,
        dtype=float,
    )

    return math.sqrt(
        float(
            np.mean(
                (actual - predicted) ** 2
            )
        )
    )


def extract_h0_plateaus(path):
    plateaus = []

    for segment in split_segments(
        load_rows(path)
    ):
        plateau = settled_plateau(
            segment
        )

        if plateau is not None:
            plateaus.append(
                plateau
            )

    if not plateaus:
        raise RuntimeError(
            f"No plateaus found in {path}"
        )

    # Normalize against the initial settled
    # (+100,+100) preload state.
    q_zero = plateaus[0]["q_deg"]

    return [
        {
            "offset":
                float(p["offset"]),

            "q_deg":
                float(p["q_deg"])
                - q_zero,

            "q_std_deg":
                float(p["q_std_deg"]),

            "m1_error_counts":
                float(
                    p["m1_error_counts"]
                ),

            "m2_error_counts":
                float(
                    p["m2_error_counts"]
                ),
        }
        for p in plateaus
    ]


def load_model_batch(path):
    rows = []

    with open(
        path,
        newline="",
    ) as f:
        reader = csv.DictReader(f)

        for row in reader:
            rows.append({
                "step":
                    int(row["step"]),

                "offset":
                    float(
                        row[
                            "offset_counts"
                        ]
                    ),

                "model_v0_q_deg":
                    float(
                        row[
                            "model_v0_q_deg"
                        ]
                    ),

                "model_v1_q_deg":
                    float(
                        row[
                            "model_v1_q_deg"
                        ]
                    ),

                "memory_state":
                    float(
                        row[
                            "model_v1_memory_state"
                        ]
                    ),
            })

    return rows


def align(
    h0,
    models,
    name,
):
    if len(h0) != len(models):
        raise RuntimeError(
            f"{name}: H0 has "
            f"{len(h0)} plateaus but "
            f"model batch has "
            f"{len(models)} steps"
        )

    records = []

    for i, (real, model) in enumerate(
        zip(h0, models)
    ):
        if not math.isclose(
            real["offset"],
            model["offset"],
            abs_tol=1e-9,
        ):
            raise RuntimeError(
                f"{name}: schedule mismatch "
                f"at step {i}: "
                f"H0={real['offset']}, "
                f"model={model['offset']}"
            )

        records.append({
            "step": i,

            "offset":
                real["offset"],

            "h0_q_deg":
                real["q_deg"],

            "model_v0_q_deg":
                model[
                    "model_v0_q_deg"
                ],

            "model_v1_q_deg":
                model[
                    "model_v1_q_deg"
                ],

            "model_v1_memory_state":
                model["memory_state"],

            "q_std_deg":
                real["q_std_deg"],

            "m1_error_counts":
                real[
                    "m1_error_counts"
                ],

            "m2_error_counts":
                real[
                    "m2_error_counts"
                ],
        })

    return records


def analyze_case(
    name,
    h0_path,
    model_path,
):
    h0 = extract_h0_plateaus(
        h0_path
    )

    models = load_model_batch(
        model_path
    )

    records = align(
        h0,
        models,
        name,
    )

    # Step zero is the normalization point,
    # so don't give either model a free zero-error
    # observation in the RMSE.
    comparison = records[1:]

    measured = [
        r["h0_q_deg"]
        for r in comparison
    ]

    model_v0 = [
        r["model_v0_q_deg"]
        for r in comparison
    ]

    model_v1 = [
        r["model_v1_q_deg"]
        for r in comparison
    ]

    v0_rmse = rmse(
        measured,
        model_v0,
    )

    v1_rmse = rmse(
        measured,
        model_v1,
    )

    improvement = (
        100.0
        * (
            v0_rmse - v1_rmse
        )
        / v0_rmse
    )

    output_dir = Path(
        h0_path
    ).parent

    csv_path = (
        output_dir
        / f"sim2real_{name}.csv"
    )

    with open(
        csv_path,
        "w",
        newline="",
    ) as f:
        fields = [
            "step",
            "offset",
            "h0_q_deg",
            "model_v0_q_deg",
            "model_v1_q_deg",
            "model_v1_memory_state",
            "q_std_deg",
            "m1_error_counts",
            "m2_error_counts",
        ]

        writer = csv.DictWriter(
            f,
            fieldnames=fields,
        )

        writer.writeheader()
        writer.writerows(records)

    plot_path = (
        output_dir
        / f"sim2real_{name}.png"
    )

    fig, ax = plt.subplots(
        figsize=(9, 6)
    )

    offsets = [
        r["offset"]
        for r in records
    ]

    ax.plot(
        offsets,
        [
            r["h0_q_deg"]
            for r in records
        ],
        "o-",
        label="Measured H0",
    )

    ax.plot(
        offsets,
        [
            r["model_v0_q_deg"]
            for r in records
        ],
        "--",
        linewidth=2,
        label="Model v0",
    )

    ax.plot(
        offsets,
        [
            r["model_v1_q_deg"]
            for r in records
        ],
        "s-",
        label="Model v1",
    )

    ax.set_xlabel(
        "Differential offset [counts]"
    )

    ax.set_ylabel(
        "Joint angle relative to initial preload [deg]"
    )

    ax.set_title(
        f"H0 sim-to-real: {name}"
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

    trajectory_plot_path = (output_dir / f"sim2real_{name}_trajectory.png")

    fig, ax = plt.subplots(figsize=(10, 5.5))

    steps = [r["step"] for r in records]

    ax.plot(steps, [r["h0_q_deg"] for r in records], "o-", label="Measured H0",)
    ax.plot(steps, [r["model_v0_q_deg"] for r in records], "--", linewidth=2, label="Model v0",)
    ax.plot(steps, [r["model_v1_q_deg"] for r in records], "s-", label="Model v1",)

    ax.set_xlabel("Experiment step")
    ax.set_ylabel("Joint angle relative to initial preload [deg]")
    ax.set_title(f"H0 sim-to-real trajectory: {name}")

    ax.grid(True, alpha=0.3,)
    ax.legend()
    fig.tight_layout()
    fig.savefig(trajectory_plot_path, dpi=200,)

    plt.close(fig)

    return {
        "name": name,

        "samples":
            len(comparison),

        "v0_rmse":
            v0_rmse,

        "v1_rmse":
            v1_rmse,

        "improvement":
            improvement,

        "csv":
            csv_path,

        "plot":
            plot_path,

        "trajectory_plot":
            trajectory_plot_path,
    }


def main():
    if len(sys.argv) != 5:
        print(
            "Usage: h0_sim2real_batch.py "
            "<quasistatic-h0.csv> "
            "<quasistatic-model.csv> "
            "<minor-loop-h0.csv> "
            "<minor-loop-model.csv>"
        )

        return 1

    quasistatic = analyze_case(
        "quasistatic",
        sys.argv[1],
        sys.argv[2],
    )

    minor = analyze_case(
        "minor_loop",
        sys.argv[3],
        sys.argv[4],
    )

    print(
        "H0 SIM-TO-REAL BATCH"
    )

    print("=" * 55)

    print()

    for result in [
        quasistatic,
        minor,
    ]:
        print(
            result["name"].upper()
        )

        print(
            f"  comparison points: "
            f"{result['samples']}"
        )

        print(
            f"  Model v0 RMSE: "
            f"{result['v0_rmse']:.3f} deg"
        )

        print(
            f"  Model v1 RMSE: "
            f"{result['v1_rmse']:.3f} deg"
        )

        print(
            f"  v1 improvement: "
            f"{result['improvement']:.1f}%"
        )

        print()

    print("Outputs:")

    print(
        quasistatic["csv"]
    )

    print(
        quasistatic["plot"]
    )

    print(
        minor["csv"]
    )

    print(
        minor["plot"]
    )

    print(
        quasistatic["trajectory_plot"]
    )

    print(
        minor["trajectory_plot"]
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
