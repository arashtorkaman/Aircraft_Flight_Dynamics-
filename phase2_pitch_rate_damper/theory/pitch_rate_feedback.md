# Pitch Rate Damper Design and Analysis

## Phase 2 – Longitudinal Flight Control System Design

---

## 1. Introduction

The objective of this phase is to design a pitch-rate damper for the longitudinal aircraft model developed in Phase 1.

A pitch-rate damper is a stability augmentation system that uses measured pitch rate to generate corrective elevator commands. Its purpose is to improve damping of the short-period mode, reduce pitch oscillations, and improve aircraft handling qualities.

This document presents the control law, closed-loop state-space formulation, root-locus design logic, MATLAB implementation, and expected engineering results.

---

## 2. Longitudinal Aircraft Model

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

The linearized longitudinal aircraft model is:

```math
\dot{x} = Ax + B\delta_e
```

where:

| Symbol | Description |
|---|---|
| `A` | Longitudinal system matrix |
| `B` | Elevator input matrix |
| `delta_e` | Elevator deflection |

---

## 3. Open-Loop Stability Analysis

The open-loop poles are obtained from:

```math
\lambda = eig(A)
```

Typical open-loop eigenvalues for the Cessna 182 longitudinal model are:

```math
\lambda_{SP} = -3.2548 \pm 1.7168j
```

```math
\lambda_{PH} = -0.0052 \pm 0.1461j
```

These poles represent two dominant longitudinal modes.

### Short-Period Mode

The short-period mode is the fast longitudinal mode. It is mainly associated with angle-of-attack and pitch-rate motion.

Typical characteristics:

- Fast response
- Higher natural frequency
- Strong pitch-rate activity
- Important for handling qualities
- Primary target of pitch-rate damping

### Phugoid Mode

The phugoid mode is the slow longitudinal mode. It mainly represents an exchange between kinetic energy and potential energy.

Typical characteristics:

- Slow oscillation
- Low natural frequency
- Light damping
- Long settling time
- Less directly affected by pitch-rate feedback

---

## 4. Motivation for Pitch-Rate Feedback

The short-period mode can produce oscillatory pitch motion. If this mode is insufficiently damped, the aircraft may feel sensitive or uncomfortable to control.

Pitch-rate feedback improves damping by commanding elevator motion that opposes rapid pitch motion.

The main benefits are:

- Reduced pitch oscillation
- Lower overshoot
- Faster settling time
- Improved short-period damping
- Better longitudinal handling qualities
- Reduced pilot workload

---

## 5. Pitch Rate Damper Control Law

The pitch-rate damper uses pitch rate as the feedback signal.

```math
\delta_e = -K_q q
```

where:

| Symbol | Description |
|---|---|
| `delta_e` | Elevator command |
| `q` | Pitch rate |
| `K_q` | Pitch-rate feedback gain |

The negative sign means the elevator command opposes the measured pitch-rate motion.

If the aircraft pitches upward too quickly, the damper commands elevator action to reduce the upward pitch rate. If the aircraft pitches downward too quickly, the controller commands the opposite elevator action.

---

## 6. State Feedback Representation

The feedback gain vector is:

```math
K = [0 \;\; 0 \;\; K_q \;\; 0]
```

The control law can be written as:

```math
\delta_e = -Kx
```

Substituting the gain vector gives:

```math
\delta_e = -[0 \;\; 0 \;\; K_q \;\; 0]x
```

Only the pitch-rate state is fed back. The forward velocity, vertical velocity, and pitch attitude states are not directly used by this controller.

---

## 7. Closed-Loop State-Space Model

Starting from the open-loop longitudinal model:

```math
\dot{x} = Ax + B\delta_e
```

Substitute the control law:

```math
\delta_e = -Kx
```

Then:

```math
\dot{x} = Ax + B(-Kx)
```

```math
\dot{x} = Ax - BKx
```

```math
\dot{x} = (A - BK)x
```

Therefore, the closed-loop system matrix is:

```math
A_{cl} = A - BK
```

This matrix governs the aircraft dynamics after pitch-rate feedback is applied.

---

## 8. Physical Interpretation

Pitch-rate feedback acts like an artificial rotational damper.

Without the controller, the aircraft damping comes only from aerodynamic stability derivatives, such as pitch-rate damping. With the controller, elevator deflection is automatically generated to oppose pitch-rate motion.

This increases the effective damping of the short-period mode.

The controller does not primarily change the trim condition. Instead, it modifies the transient response of the aircraft.

---

## 9. Root-Locus Design

Root-locus analysis is used to examine how the closed-loop poles move as the pitch-rate feedback gain changes.

For each value of `K_q`, the closed-loop matrix is:

```math
A_{cl}(K_q) = A - BK
```

The corresponding closed-loop poles are:

```math
\lambda_{cl} = eig(A_{cl})
```

As `K_q` increases, the expected behavior is:

- Short-period poles move farther left in the complex plane.
- Short-period damping ratio increases.
- Settling time decreases.
- Oscillations decay faster.
- Phugoid poles remain mostly unchanged.

The phugoid mode is less affected because pitch-rate feedback mainly targets the fast pitch dynamics.

---

## 10. Gain Selection Procedure

A practical gain selection process is:

### Step 1: Start with the open-loop case

```math
K_q = 0
```

This gives the original open-loop aircraft dynamics.

### Step 2: Increase the gain gradually

Evaluate the closed-loop poles for a range of gains, for example:

```math
0 \leq K_q \leq 5
```

### Step 3: Monitor the response

For each value of `K_q`, examine:

- Pole locations
- Damping ratio
- Natural frequency
- Settling time
- Overshoot
- Elevator control effort

### Step 4: Select a practical gain

Choose a gain that improves damping without creating unrealistic actuator activity.

A good gain should:

- Improve short-period damping
- Reduce oscillations
- Avoid excessive elevator deflection
- Avoid actuator saturation
- Avoid excessive sensitivity to sensor noise

---

## 11. MATLAB Implementation

### Closed-Loop Model

```matlab
Kq = 1.0;

K = [0 0 Kq 0];

Acl = A - B*K;

sys_ol = ss(A,B,C,D);
sys_cl = ss(Acl,B,C,D);

poles_ol = eig(A);
poles_cl = eig(Acl);
```

### Pole Comparison

```matlab
figure;
plot(real(poles_ol), imag(poles_ol), 'x', 'LineWidth', 2);
hold on;
plot(real(poles_cl), imag(poles_cl), 'o', 'LineWidth', 2);
grid on;
xlabel('Real Axis');
ylabel('Imaginary Axis');
title('Open-Loop vs Closed-Loop Poles');
legend('Open Loop', 'Closed Loop');
```

### Gain Sweep

```matlab
Kq_values = 0:0.1:5;

figure;
hold on;
grid on;

for Kq = Kq_values
    K = [0 0 Kq 0];
    Acl = A - B*K;
    poles = eig(Acl);
    plot(real(poles), imag(poles), 'b.');
end

xlabel('Real Axis');
ylabel('Imaginary Axis');
title('Pitch Rate Damper Gain Sweep');
```

### Step Response Comparison

```matlab
figure;
step(sys_ol);
hold on;
step(sys_cl);
grid on;
legend('Open Loop', 'Closed Loop');
title('Open-Loop vs Closed-Loop Step Response');
```

---

## 12. Expected Results

The pitch-rate damper should improve the aircraft short-period response.

Expected trends:

| Metric | Open Loop | Closed Loop |
|---|---|---|
| Overshoot | Higher | Lower |
| Settling time | Longer | Shorter |
| Short-period damping | Lower | Higher |
| Pitch oscillation | More visible | Reduced |
| Handling quality | Less favorable | Improved |

---

## 13. Engineering Discussion

Pitch-rate feedback is commonly used as an inner-loop stability augmentation system.

It is effective because pitch rate is directly related to the short-period mode. By feeding back pitch rate to the elevator, the controller adds damping without requiring a complex control architecture.

This type of controller is relevant to:

- Stability augmentation systems
- UAV autopilots
- Fly-by-wire control laws
- Aircraft pitch-attitude hold systems
- Inner-loop longitudinal control systems

The pitch-rate damper is often one of the first control loops implemented before adding outer-loop functions such as pitch-attitude hold, altitude hold, or glide-slope tracking.

---

## 14. Files and Figures to Include

Recommended Phase 2 project files:

```text
phase2_pitch_rate_damper/
│
├── theory/
│   └── pitch_rate_feedback.md
│
├── matlab/
│   ├── pitch_rate_damper.m
│   └── gain_sweep_analysis.m
│
├── figures/
│   ├── open_loop_poles.png
│   ├── closed_loop_poles.png
│   ├── gain_sweep_poles.png
│   ├── open_loop_step_response.png
│   └── closed_loop_step_response.png
│
└── README.md
```

---

## 15. Conclusion

A pitch-rate damper was developed for the longitudinal aircraft model.

The controller uses the feedback law:

```math
\delta_e = -K_q q
```

The resulting closed-loop dynamics are governed by:

```math
A_{cl} = A - BK
```

The pitch-rate damper increases short-period damping, reduces oscillatory pitch motion, and improves longitudinal handling qualities.

This phase represents the first closed-loop flight-control design in the aerospace engineering portfolio and provides the foundation for later autopilot functions such as pitch-attitude hold, altitude hold, and automatic landing control.

---

## References

1. Stevens, B. L., Lewis, F. L., and Johnson, E. N., *Aircraft Control and Simulation*.
2. Nelson, R. C., *Flight Stability and Automatic Control*.
3. Etkin, B., *Dynamics of Atmospheric Flight*.
4. Cook, M. V., *Flight Dynamics Principles*.
5. MIL-F-8785C Flying Qualities Specification.
