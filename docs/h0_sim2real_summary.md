# H0 Sim-to-Real Summary

## Objective

Compare the original nominal model (Model v0) and the identified stateful model (Model v1) against the realized H0 antagonistic tendon-driven compliant joint.

## Hardware platform

- 1-DOF antagonistic tendon-driven compliant joint
- AS5600 joint encoder
- two motor-side encoders
- common-mode preload and differential tendon actuation
- commissioned H0 configuration used throughout experiments

## Models

### Model v0
Nominal physics-derived model with memoryless quasi-static behavior.

### Model v1
Empirical stateful model with:
- static gain term
- bias term
- continuous hysteresis memory state
- first-order relaxation dynamics

## Experimental datasets

### 1. Quasi-static sweep
- common preload fixed
- differential offsets swept over the safe range
- 5 s dwell per measurement point

### 2. Minor-loop experiment
- same preload
- includes small reversals:
  - +20 → +18 → +20
  - -20 → -18 → -20

## Key identified results

### Quasi-static characterization
- Model v0 RMSE: 1.364 deg
- Model v1 RMSE: 0.107 deg
- Improvement: 92.2%

### Minor-loop validation
- Model v0 RMSE: 1.256 deg
- Model v1 RMSE: 0.217 deg
- Improvement: 82.8%

## Interpretation

Model v0 fails because it assumes a memoryless relationship between commanded differential tendon displacement and joint angle.

H0 exhibits:
- hysteresis / path dependence
- partial reversal behavior
- short-timescale relaxation

Model v1 captures these effects with a compact internal-state formulation and significantly improves sim-to-real agreement.

## Main conclusion

The realized H0 joint exposes a clear and reproducible sim-to-real discrepancy relative to the nominal model. A small stateful extension (Model v1) dramatically improves prediction quality and correctly captures minor-loop behavior that Model v0 cannot represent.

## Canonical figures

- `results/h0/sim2real_quasistatic.png`
- `results/h0/sim2real_quasistatic_trajectory.png`
- `results/h0/sim2real_minor_loop.png`
- `results/h0/sim2real_minor_loop_trajectory.png`

## Canonical scripts

- `analysis/h0_quasistatic.py`
- `analysis/h0_relaxation.py`
- `analysis/h0_dynamic_validation.py`
- `analysis/h0_minor_loop_validation.py`
- `analysis/h0_sim2real_batch.py`