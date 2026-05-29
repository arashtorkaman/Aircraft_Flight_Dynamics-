# Closed-Loop Longitudinal Model

## Phase 2 - Pitch Rate Damper

---

## 1. Purpose

This document defines the closed-loop longitudinal aircraft model used for the Phase 2 pitch-rate damper design.

The open-loop aircraft model was developed in Phase 1. Phase 2 adds pitch-rate feedback to improve damping of the short-period mode.

The pitch-rate damper uses elevator deflection to oppose pitch-rate motion. This improves transient response, reduces oscillation, and provides a first stability augmentation system for the aircraft model.

---

## 2. Open-Loop Longitudinal Model

The longitudinal state vector is defined as:

```math
x = [u,\; w,\; q,\; \theta]^T
```

where:

| State | Description |
|---|---|
| `u` | Forward velocity perturbation |
| `w` | Vertical velocity perturbation |
| `q` | Pitch rate |
| `theta` | Pitch attitude |

The open-loop state-space model is:

```math
\dot{x}=Ax+B\delta_e
```

where:

| Symbol | Description |
|---|---|
| `A` | Longitudinal aircraft system matrix |
| `B` | Elevator input matrix |
| `delta_e` | Elevator deflection |

The output equation is:

```math
y=Cx+D\delta_e
```

---

## 3. Pitch-Rate Feedback Control Law

The pitch-rate damper feedback law is:

```math
\delta_e=-K_q q
```

where:

| Symbol | Description |
|---|---|
| `K_q` | Pitch-rate feedback gain |
| `q` | Pitch rate |
| `delta_e` | Elevator command |

The negative sign means that the elevator command opposes pitch-rate motion.

For positive pitch rate, the controller commands elevator action that reduces the pitch-up rotation. For negative pitch rate, the controller commands elevator action that reduces the pitch-down rotation.

---

## 4. Feedback Gain Vector

The feedback vector is written as:

```math
K=[0\;\;0\;\;K_q\;\;0]
```

The control law can then be written compactly as:

```math
\delta_e=-Kx
```

Since only the pitch-rate state is fed back, the controller does not directly use forward velocity, vertical velocity, or pitch attitude.

Expanded form:

```math
\delta_e=-[0\;\;0\;\;K_q\;\;0]x
```

---

## 5. Closed-Loop Derivation

Start with the open-loop model:

```math
\dot{x}=Ax+B\delta_e
```

Substitute the pitch-rate feedback law:

```math
\delta_e=-Kx
```

Then:

```math
\dot{x}=Ax+B(-Kx)
```

```math
\dot{x}=Ax-BKx
```

```math
\dot{x}=(A-BK)x
```

Therefore, the closed-loop system matrix is:

```math
A_{cl}=A-BK
```

The closed-loop aircraft model is:

```math
\dot{x}=A_{cl}x
```

For forced-response simulation with an external elevator command input, the model can be written as:

```math
\dot{x}=A_{cl}x+B\delta_{cmd}
```

where `delta_cmd` is an external elevator command or reference input used for simulation.

---

## 6. Closed-Loop Output Equation

The output equation can remain:

```math
y=Cx+D\delta_{cmd}
```

Common outputs for Phase 2 are:

| Output | Purpose |
|---|---|
| `u` | Forward velocity response |
| `w` | Vertical velocity response |
| `q` | Pitch-rate response |
| `theta` | Pitch-attitude response |
| `delta_e` | Elevator command response |

For pitch-rate damper validation, the most important outputs are `q`, `theta`, and elevator deflection.

---

## 7. MATLAB Implementation

```matlab
% Pitch-rate feedback gain
Kq = 1.0;

% Feedback gain vector
K = [0 0 Kq 0];

% Closed-loop system matrix
Acl = A - B*K;

% Open-loop and closed-loop systems
sys_ol = ss(A,B,C,D);
sys_cl = ss(Acl,B,C,D);

% Pole calculation
poles_ol = eig(A);
poles_cl = eig(Acl);
```

---

## 8. Engineering Interpretation

The pitch-rate damper modifies the aircraft dynamics by adding artificial damping to the pitch axis.

The main effect is expected in the short-period mode because this mode is strongly associated with angle-of-attack and pitch-rate motion.

Expected closed-loop improvements:

- Short-period poles move farther into the left-half plane.
- Damping ratio increases.
- Oscillations decay faster.
- Pitch-rate response becomes smoother.
- Settling time decreases.

The phugoid mode is expected to remain mostly unchanged because it is slower and dominated by energy exchange between altitude and airspeed.

---

## 9. Verification Checklist

Before using the closed-loop model for final results, verify the following:

- The dimensions of `A`, `B`, and `K` are compatible.
- The closed-loop matrix `Acl = A - B*K` is computed correctly.
- All closed-loop poles remain in the left-half plane.
- The selected gain does not create unrealistic elevator commands.
- Closed-loop response is better damped than open-loop response.
- The short-period mode improves without destabilizing the phugoid mode.

---

## 10. Conclusion

The closed-loop longitudinal model was derived by applying pitch-rate feedback to the elevator input.

The resulting closed-loop matrix is:

```math
A_{cl}=A-BK
```

This model forms the basis for Phase 2 root-locus analysis, gain tuning, and open-loop versus closed-loop response comparison.
