# Pitch Rate Damper Design and Analysis

## Phase 2 – Longitudinal Flight Control System Design

---

# 1. Introduction

The objective of this phase is to design a Pitch Rate Damper for the longitudinal aircraft model developed in Phase 1.

A pitch-rate damper is one of the simplest stability augmentation systems used in aerospace control engineering. The controller uses measured pitch rate to generate corrective elevator commands that improve aircraft damping and reduce oscillatory behavior.

The primary objective is to improve the short-period dynamic mode while preserving the overall longitudinal characteristics of the aircraft.

---

# 2. Longitudinal Aircraft Model

The longitudinal state vector is defined as:

$$
x = [u,; w,; q,; \theta]^T
$$

where:

| State    | Description                    |
| -------- | ------------------------------ |
| $u$      | Forward velocity perturbation  |
| $w$      | Vertical velocity perturbation |
| $q$      | Pitch rate                     |
| $\theta$ | Pitch attitude                 |

The linearized longitudinal equations of motion are:

$$
\dot{x}=Ax+B\delta_e
$$

where:

* $A$ = longitudinal system matrix
* $B$ = control input matrix
* $\delta_e$ = elevator deflection

---

# 3. Open-Loop Stability Analysis

The open-loop poles are obtained from:

$$
\lambda = eig(A)
$$

Typical eigenvalues for the Cessna 182 model are:

$$
\lambda_{SP}
============

-3.2548
\pm
1.7168j
$$

$$
\lambda_{PH}
============

-0.0052
\pm
0.1461j
$$

These poles represent two dominant longitudinal modes:

## Short-Period Mode

Characteristics:

* Fast dynamics
* Dominated by angle of attack and pitch rate
* Strong influence on handling qualities
* Usually well damped

## Phugoid Mode

Characteristics:

* Slow oscillation
* Exchange of kinetic and potential energy
* Lightly damped
* Long settling time

---

# 4. Motivation for Pitch-Rate Feedback

The short-period mode may exhibit oscillatory behavior that reduces handling quality.

Pitch-rate feedback provides artificial damping by opposing rapid pitch motion.

Benefits include:

* Reduced overshoot
* Improved damping ratio
* Faster settling time
* Improved flying qualities
* Reduced pilot workload

---

# 5. Pitch Rate Damper Control Law

The controller uses pitch rate as the feedback signal.

The control law is:

$$
\delta_e = -K_q q
$$

where:

| Symbol     | Description              |
| ---------- | ------------------------ |
| $\delta_e$ | Elevator command         |
| $q$        | Pitch rate               |
| $K_q$      | Pitch-rate feedback gain |

The negative sign indicates that the elevator opposes the measured pitch motion.

---

# 6. State Feedback Representation

The feedback gain vector is:

$$
K = [0 ;; 0 ;; K_q ;; 0]
$$

The control law becomes:

$$
\delta_e = -Kx
$$

Since only pitch rate is fed back,

$$
\delta_e
========

-[0 ;; 0 ;; K_q ;; 0]x
$$

This controller only affects the pitch-rate state.

---

# 7. Closed-Loop State-Space Model

Starting with the open-loop system:

$$
\dot{x}=Ax+B\delta_e
$$

Substituting the feedback law:

$$
\delta_e=-Kx
$$

gives:

$$
\dot{x}=Ax-BKx
$$

Factoring out the state vector:

$$
\dot{x}=(A-BK)x
$$

The closed-loop system matrix becomes:

$$
A_{cl}=A-BK
$$

This matrix governs the closed-loop aircraft dynamics.

---

# 8. Physical Interpretation

Pitch-rate feedback introduces additional damping into the aircraft.

When the aircraft rotates upward too quickly:

* Positive pitch rate is detected.
* Elevator deflection opposes the motion.
* The oscillation is reduced.

When the aircraft rotates downward too quickly:

* Negative pitch rate is detected.
* Elevator response again opposes the motion.

This behavior is analogous to a rotational damper in a mechanical system.

---

# 9. Root-Locus Design

The gain $K_q$ is varied to examine the movement of the closed-loop poles.

The closed-loop poles are computed from:

$$
\lambda_{cl}
============

eig(A-BK)
$$

As $K_q$ increases:

* Short-period poles move further left.
* Damping ratio increases.
* Settling time decreases.
* Stability margins improve.

The phugoid poles remain largely unchanged because the controller primarily affects pitch-rate dynamics.

---

# 10. Gain Selection

A practical gain selection procedure is:

### Step 1

Begin with:

$$
K_q = 0
$$

This corresponds to the open-loop system.

### Step 2

Gradually increase gain.

Monitor:

* Pole locations
* Damping ratio
* Settling time
* Control effort

### Step 3

Choose a gain that:

* Increases damping
* Produces smooth responses
* Avoids actuator saturation
* Avoids excessive control activity

Typical investigation range:

$$
0 \le K_q \le 5
$$

---

# 11. MATLAB Implementation

```matlab
Kq = 1.0;

K = [0 0 Kq 0];

Acl = A - B*K;

sys_ol = ss(A,B,C,D);
sys_cl = ss(Acl,B,C,D);

poles_ol = eig(A);
poles_cl = eig(Acl);
```

---

## Root Locus

```matlab
Kvec = 0:0.1:5;

figure;
rlocus(ss(A,B,[0 0 1 0],0));
grid on;
title('Pitch Rate Damper Root Locus');
```

---

## Step Response Comparison

```matlab
figure;
step(sys_ol);
hold on;
step(sys_cl);

legend('Open Loop','Closed Loop');
grid on;

title('Open-Loop vs Closed-Loop Response');
```

---

# 12. Expected Results

The pitch-rate damper should produce:

* Improved short-period damping
* Reduced oscillations
* Faster settling
* Lower overshoot
* More stable pitch response

Expected observations:

| Metric           | Open Loop | Closed Loop |
| ---------------- | --------- | ----------- |
| Overshoot        | Higher    | Lower       |
| Settling Time    | Longer    | Shorter     |
| Damping Ratio    | Lower     | Higher      |
| Stability Margin | Smaller   | Larger      |

---

# 13. Engineering Discussion

Pitch-rate feedback is widely used in aerospace flight-control systems.

Examples include:

* Stability augmentation systems
* Fly-by-wire aircraft
* UAV autopilots
* Military fighter aircraft
* Commercial transport aircraft

In modern control architectures, pitch-rate feedback is commonly used as an inner-loop controller due to its effectiveness and simplicity.

---

# 14. Conclusion

A pitch-rate damper was developed using pitch-rate feedback.

The control law:

$$
\delta_e = -K_q q
$$

introduces artificial damping that improves the short-period dynamics of the aircraft.

The closed-loop dynamics are described by:

$$
A_{cl}=A-BK
$$

Analysis demonstrates improved damping, reduced oscillations, and enhanced handling qualities.

This phase represents the first closed-loop flight-control system in the aerospace engineering portfolio and provides the foundation for more advanced autopilot and flight-control designs.

---

# References

1. Stevens, B. L., Lewis, F. L., and Johnson, E. N., *Aircraft Control and Simulation*.
2. Nelson, R. C., *Flight Stability and Automatic Control*.
3. Etkin, B., *Dynamics of Atmospheric Flight*.
4. Cook, M. V., *Flight Dynamics Principles*.
5. MIL-F-8785C Flying Qualities Specification.

