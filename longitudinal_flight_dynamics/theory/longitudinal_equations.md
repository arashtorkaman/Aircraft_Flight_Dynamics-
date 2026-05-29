# Phase 1 Theory — Longitudinal Aerodynamics of the Cessna 182

## 1. Purpose

This document summarizes the theoretical basis for the **Phase 1 longitudinal dynamics model** of the **Cessna 182** at a cruise operating condition. The objective of Phase 1 is to move from aerodynamic derivatives to a usable linear state-space model, then analyze the resulting longitudinal modes and basic feedback structure.

The model supports:

- open-loop longitudinal stability analysis,
- modal identification of the **short-period** and **phugoid** modes,
- time-domain response studies,
- preliminary pitch-rate feedback design.

---

## 2. Scope and modeling assumptions

The Phase 1 model is a **linearized longitudinal perturbation model** about a steady cruise trim point.

### Assumptions

1. The aircraft is treated as a rigid body.
2. Motion is restricted to the **longitudinal plane**.
3. Small perturbations are assumed about the trim condition.
4. Aerodynamic derivatives are constant near the chosen operating point.
5. Cruise trim is approximated as **level flight**.
6. Imperial units are used consistently.

### Trim condition used

Cruise trim angle assumption:

$$
\theta_0 \approx 0
$$

Cruise trim speed:

$$
U_0 = 220.1\ \text{ft/s}
$$

Gravitational acceleration:

$$
g = 32.174\ \text{ft/s}^2
$$

---

## 3. Longitudinal states and input

The longitudinal state vector is defined as:

$$
x = \begin{bmatrix} u & \alpha & q & \theta \end{bmatrix}^T
$$

where:

- $u$: perturbation in forward speed,
- $\alpha$: perturbation in angle of attack,
- $q$: pitch rate,
- $\theta$: pitch attitude.

The control input is the elevator deflection:

$$
\delta_e
$$

---

## 4. Nondimensional longitudinal aerodynamic coefficients

For the cruise condition, the Cessna 182 longitudinal aerodynamic coefficients are:

### Steady-state coefficients

$$
C_{L1} = 0.307, \qquad C_{D1} = 0.032, \qquad C_{m1} = 0
$$

$$
C_{TX1} = 0.032, \qquad C_{mT1} = 0
$$

### Stability derivatives

$$
C_{D0} = 0.027, \qquad C_{Du} = 0, \qquad C_{D\alpha} = 0.121, \qquad C_{TXu} = -0.096
$$

$$
C_{L0} = 0.307, \qquad C_{Lu} = 0, \qquad C_{L\alpha} = 4.41
$$

$$
C_{L\dot{\alpha}} = 1.7, \qquad C_{Lq} = 3.9
$$

$$
C_{m0} = 0.04, \qquad C_{mu} = 0, \qquad C_{m\alpha} = -0.613
$$

$$
C_{m\dot{\alpha}} = -7.27, \qquad C_{mq} = -12.4
$$

$$
C_{mTu} = 0, \qquad C_{mT\alpha} = 0
$$

### Control derivatives

$$
C_{D\delta_e} = 0, \qquad C_{L\delta_e} = 0.43, \qquad C_{m\delta_e} = -1.122
$$

These coefficients describe how aerodynamic lift, drag, and pitching moment vary with angle of attack, pitch rate, and elevator deflection near the cruise trim point.

---

## 5. Dimensional longitudinal stability derivatives

The dimensional longitudinal derivatives used in the cruise model are:

### Axial force derivatives

$$
X_u = -0.0304, \qquad X_{Tu} = -0.0152, \qquad X_\alpha = 19.459, \qquad X_{\delta_e} = 0
$$

The total speed derivative in the axial direction is:

$$
X_u^{\mathrm{tot}} = X_u + X_{Tu} = -0.0456
$$

### Normal force derivatives

$$
Z_u = -0.2919, \qquad Z_\alpha = -464.71
$$

$$
Z_{\dot{\alpha}} = -1.98, \qquad Z_q = -4.542, \qquad Z_{\delta_e} = -44.985
$$

### Pitching moment derivatives

$$
M_u = 0, \qquad M_{Tu} = 0, \qquad M_\alpha = -19.26
$$

$$
M_{\dot{\alpha}} = -2.543, \qquad M_q = -4.337, \qquad M_{\delta_e} = -35.251
$$

Since thrust moment effects in speed are zero in this dataset:

$$
M_u^{\mathrm{tot}} = M_u + M_{Tu} = 0
$$

---

## 6. Descriptor-form state-space equations

Because the model includes $Z_{\dot{\alpha}}$ and $M_{\dot{\alpha}}$, the longitudinal equations are first written in descriptor form:

$$
E\dot{x} = Ax + B\delta_e
$$

with:

$$
E =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & U_0 - Z_{\dot{\alpha}} & 0 & 0 \\
0 & -M_{\dot{\alpha}} & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Substituting the cruise values:

$$
E =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 222.08 & 0 & 0 \\
0 & 2.543 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

The system matrix is:

$$
A =
\begin{bmatrix}
X_u^{\mathrm{tot}} & X_\alpha & 0 & -g \\
Z_u & Z_\alpha & U_0 + Z_q & 0 \\
M_u^{\mathrm{tot}} & M_\alpha & M_q & 0 \\
0 & 0 & 1 & 0
\end{bmatrix}
$$

Numerically,

$$
A =
\begin{bmatrix}
-0.0456 & 19.459 & 0 & -32.174 \\
-0.2919 & -464.71 & 215.558 & 0 \\
0 & -19.26 & -4.337 & 0 \\
0 & 0 & 1 & 0
\end{bmatrix}
$$

The input matrix is:

$$
B =
\begin{bmatrix}
X_{\delta_e} \\
Z_{\delta_e} \\
M_{\delta_e} \\
0
\end{bmatrix}
=
\begin{bmatrix}
0 \\
-44.985 \\
-35.251 \\
0
\end{bmatrix}
$$

---

## 7. Standard state-space form

The standard state-space form is obtained through:

$$
\dot{x} = A_{ss}x + B_{ss}\delta_e
$$

where:

$$
A_{ss} = E^{-1}A, \qquad B_{ss} = E^{-1}B
$$

For the chosen trim condition, the resulting numerical matrices are:

$$
A_{ss} =
\begin{bmatrix}
-0.0456 & 19.4590 & 0 & -32.1740 \\
-0.001314 & -2.092534 & 0.970632 & 0 \\
0.003342 & -13.938686 & -6.805318 & 0 \\
0 & 0 & 1 & 0
\end{bmatrix}
$$

$$
B_{ss} =
\begin{bmatrix}
0 \\
-0.202562 \\
-34.735884 \\
0
\end{bmatrix}
$$

A full-state output model can be defined as:

$$
C = I_{4 \times 4}, \qquad D = 0_{4 \times 1}
$$

---

## 8. Open-loop longitudinal modes

The open-loop poles are the eigenvalues of $A_{ss}$:

$$
\lambda_{1,2} = -4.4497 \pm 2.8249i
$$

$$
\lambda_{3,4} = -0.0220 \pm 0.1698i
$$

These correspond to the two classical longitudinal modes.

### 8.1 Short-period mode

The fast complex pair

$$
-4.4497 \pm 2.8249i
$$

is the **short-period mode**.

Its approximate characteristics are:

$$
\omega_{n,sp} \approx 5.27\ \text{rad/s}
$$

$$
\zeta_{sp} \approx 0.844
$$

This mode is dominated primarily by **angle of attack** and **pitch rate**. It is relatively fast and well damped.

### 8.2 Phugoid mode

The slow complex pair

$$
-0.0220 \pm 0.1698i
$$

is the **phugoid mode**.

Its approximate characteristics are:

$$
\omega_{n,ph} \approx 0.171\ \text{rad/s}
$$

$$
\zeta_{ph} \approx 0.129
$$

This mode is dominated primarily by **forward speed** and **pitch attitude**. It is slower and much more lightly damped than the short-period mode.

---

## 9. Time-domain interpretation

Open-loop elevator step simulations over different time horizons help separate the two modes.

### 10-second simulation window

A short simulation window emphasizes the **short-period mode**:

- rapid transient behavior,
- strongest in $\alpha$ and $q$,
- relatively fast decay.

### 200-second simulation window

A long simulation window reveals the **phugoid mode**:

- slow oscillation,
- strongest in $u$ and $\theta$,
- lightly damped energy exchange between speed and altitude-related attitude response.

Together, these two windows provide a clear modal interpretation of the aircraft's longitudinal dynamics.

---

## 10. Pitch-rate feedback theory

A natural next step in longitudinal control design is to introduce a **pitch-rate damper** using elevator feedback.

The control law is written as:

$$
\delta_e = \delta_{cmd} + K_q q
$$

where:

- $\delta_{cmd}$ is the elevator command,
- $q$ is the measured pitch rate,
- $K_q$ is the pitch-rate feedback gain.

Using the output selector

$$
C_q = \begin{bmatrix} 0 & 0 & 1 & 0 \end{bmatrix}
$$

we have:

$$
q = C_q x
$$

Substituting into the state equation yields the closed-loop model:

$$
\dot{x} = A_{cl}x + B\delta_{cmd}
$$

with

$$
A_{cl} = A + B K_q C_q
$$

This feedback is used to shift the short-period poles and improve damping of the longitudinal response.

---

## 11. Root-locus interpretation

The elevator-to-pitch-rate transfer function is:

$$
G_q(s) = \frac{q(s)}{\delta_e(s)} = C_q(sI - A)^{-1}B
$$

In root-locus design, the gain $K_q$ is varied to observe how the closed-loop poles move in the complex plane.

A suitable gain is chosen so that:

- the short-period poles move farther left,
- damping improves,
- the phugoid remains acceptable,
- the elevator demand remains physically reasonable.

For the current linear model, a higher gain such as

$$
K_q = 5
$$

produced a strongly damped closed-loop response while keeping elevator demand modest, indicating that the controller is effective within the assumptions of the model.

---

## 12. Engineering interpretation of Phase 1

Phase 1 establishes that the Cessna 182 longitudinal dynamics can be captured by a physically meaningful linear model around the cruise trim point. The model reproduces the expected classical longitudinal behavior:

- a **fast, well-damped short-period mode**,
- a **slow, lightly damped phugoid mode**,
- stable open-loop dynamics,
- improved transient behavior under pitch-rate feedback.

This provides a solid foundation for subsequent phases involving:

- closed-loop control refinement,
- additional performance metrics,
- nonlinear modeling,
- landing and autopilot extensions,
- Simulink implementation.

---

## 13. Suggested companion files for Phase 1

To keep the repository clean, this file should sit alongside:

- `model_assumptions.md`
- `validation_plan.md`
- `results_summary.md`
- MATLAB scripts for state-space generation and simulation
- figures for eigenvalues, root locus, Bode plots, and time responses

---

## 14. Summary

Phase 1 converts published Cessna 182 longitudinal aerodynamic data into a complete linear longitudinal model suitable for control-oriented analysis. The resulting state-space formulation supports eigenvalue analysis, modal identification, time-domain simulation, and preliminary pitch-rate feedback design. This phase is the theoretical and analytical foundation for the rest of the project.

