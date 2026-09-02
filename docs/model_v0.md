# Antagonistic Tendon-Driven Compliant Joint &mdash; Model v0

## Purpose

This document defines the initial dynamic model of a one-degree-of-freedom antagonistic tendon-driven compliant joint. The model will serve as the initial plant model for simulation, control, system identification, and later comparison with physical hardware.

## Mechanism

The mechanism consists of:

- a rigid fixed base,
- a rigid moving link,
- a revolute joint,
- a joint drum or pulley,
- two motor-driven spools,
- two tendons, and
- one linear spring in each tendon path.

The two tendons act antagonistically: tendon 1 generates torque in the positive joint direction, while tendon 2 generates torque in the negative joint direction.

## Sign Convention

Positive $q$ denotes counter-clockwise joint rotation.

Positive $\theta_1$ and $\theta_2$ denote motor spool rotation that winds in the corresponding tendon and increases its extension.

Under this convention, increasing $q$ decreases the extension of tendon 1 and increases the extension of tendon 2.

## Variables

| Symbol | Meaning | Unit |
|:---:|---|:---:|
| $q$ | joint angle | $\mathrm{rad}$ |
| $\dot{q}$ | joint angular velocity | $\mathrm{rad/s}$ |
| $\ddot{q}$ | joint angular acceleration | $\mathrm{rad/s^2}$ |
| $\theta_1$ | motor 1 spool angle | $\mathrm{rad}$ |
| $\theta_2$ | motor 2 spool angle | $\mathrm{rad}$ |
| $r_j$ | joint tendon moment arm | $\mathrm{m}$ |
| $r_m$ | motor spool radius | $\mathrm{m}$ |
| $k$ | spring stiffness | $\mathrm{N/m}$ |
| $I$ | rotational inertia of the moving link | $\mathrm{kg\cdot m^2}$ |
| $\delta_0$ | initial spring extension | $\mathrm{m}$ |
| $b$ | rotational viscous damping coefficient | $\mathrm{N \cdot m \cdot s/rad}$ | 
| $\delta_1$ | spring 1 extension | $\mathrm{m}$ |
| $\delta_2$ | spring 2 extension | $\mathrm{m}$ |
| $F_1$ | tendon 1 force | $\mathrm{N}$ |
| $F_2$ | tendon 2 force | $\mathrm{N}$ |

## Assumptions

The model makes the following simplifying assumptions:

- The base and moving link are rigid.
- The tendons are massless and inextensible except for the modeled springs.
- Both springs are identical and linear, with stiffness $k$.
- Both tendons remain taut throughout the modeled motion.
- The motor spool radius $r_m$ and joint moment arm $r_j$ are constant.
- The motor spool angles $\theta_1$ and $\theta_2$ are prescribed inputs; motor dynamics are not modeled.
- Joint damping is represented by linear viscous damping.
- Gravity, Coulomb friction, backlash, tendon-routing losses, external loads, and sensor noise are neglected.
- Because the tendons can transmit tension but not compression, this model is valid only while both spring extensions remain positive, $\delta_1 > 0$ and $\delta_2 > 0$. Tendon slack will be treated as a later model extension rather than included in Model v0.

## Input Decomposition

The actuator inputs may be decomposed into a differential component

$$
\theta_d = \theta_1 - \theta_2
$$

and a common-mode component

$$
\theta_c = \frac{\theta_1 + \theta_2}{2}.
$$

The joint dynamics can therefore be written as

$$
I\ddot{q} + b\dot{q} + 2kr_j^2 q
=
kr_jr_m\theta_d.
$$

Under the symmetric linear-spring assumptions of Model v0, only the differential input $\theta_d$ affects the joint motion.

The common-mode input $\theta_c$ changes the individual tendon tensions but does not change the effective rotational stiffness

$$
K_q = 2kr_j^2.
$$

Therefore, Model v0 predicts that symmetric co-contraction increases internal tendon tension without changing joint stiffness.

## Tendon Extension

The extension of tendon 1 is

$$
\delta_1 = \delta_0 + r_m \theta_1 - r_j q.
$$

$$
\delta_2 = \delta_0 + r_m \theta_2 + r_j q.
$$

A positive joint rotation therefore unloads tendon 1 and stretches tendon 2.

## Tendon Forces

Assuming linear springs and taut tendons, Hooke's law gives

$$
F_1 = k \delta_1,
$$

and

$$
F_2 = k \delta_2.
$$

## Tendon Torque

Tendon 1 produces positive joint torque,

$$
\tau_1 = r_j F_1,
$$

while tendon 2 produces negative joint torque,

$$
\tau_2 = - r_j F_2.
$$

The net tendon torque is therefore

$$
\tau_{\mathrm{tendon}} = r_j(F_1 - F_2).
$$

## Joint Dynamics

Applying rotational Newtonian dynamics gives

$$
I\ddot{q} = \tau_{\mathrm{tendon}} - b \dot{q}.
$$

Substituting the tendon forces,

$$
I\ddot{q}
=
r_j
\left[
k(\delta_0+r_m\theta_1-r_jq)
-
k(\delta_0+r_m\theta_2+r_jq)
\right]
-
b\dot{q}.
$$

After simplification,

$$
I\ddot{q} = k r_j r_m (\theta_1 - \theta_2) - 2 k r_j^2q - b \dot{q}.
$$

The initial spring extension $\delta_0$ cancels from the net joint torque under the symmetric linear-spring assumptions of this model. It still affects the individual tendon tensions.

The effective rotational spring stiffness of the joint in this model is

$$
K_q = 2 k r_j^2.
$$

## Numerical Sanity Check

Consider the following parameter values:

| Symbol | Value |
|---|---:|
| $\theta_1$ | $1\,\mathrm{rad}$ |
| $\theta_2$ | $0\,\mathrm{rad}$ |
| $r_j$ | $0.02\,\mathrm{m}$ |
| $r_m$ | $0.01\,\mathrm{m}$ |
| $k$ | $200\,\mathrm{N/m}$ |
| $I$ | $0.01\,\mathrm{kg \cdot m^2}$ |
| $\delta_0$ | $0.01\,\mathrm{m}$ |
| $b$ | $0.02\,\mathrm{N \cdot m \cdot s / rad}$ |

At static equilibrium,

$$
\dot{q} = 0,
$$

and

$$
\ddot{q} = 0.
$$

Therefore,

$$
k r_j r_m (\theta_1 - \theta_2) = 2 k r_j^2 q_{\mathrm{eq}}.
$$

Solving for the equilibrium joint angle gives

$$
q_{\mathrm{eq}} = \frac{r_m}{2r_j} (\theta_1 - \theta_2).
$$

Substituting the numerical values,

$$
q_{\mathrm{eq}} = \frac{0.01}{2(0.02)} (1 - 0) = 0.25\,\mathrm{rad}.
$$

Thus,

$$
0.25\,\mathrm{rad} \approx 14.3^\circ.
$$

The corresponding spring extensions are

$$
\delta_1 = 0.01 + (0.01)(1) - (0.02)(0.25) = 0.015\,\mathrm{m},
$$

and

$$
\delta_2 = 0.01 + (0.01)(0) + (0.02)(0.25) = 0.015\,\mathrm{m}.
$$

Therefore,

$$
F_1 = F_2 = 200(0.015) = 3\,\mathrm{N}.
$$

The resulting net tendon torque is

$$
\tau_{\mathrm{tendon}} = r_j(F_1 - F_2) = 0,
$$

which is consistent with the assumed static equilibrium.
