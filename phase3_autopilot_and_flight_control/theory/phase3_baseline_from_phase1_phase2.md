# Phase 3 — Baseline Reconstructed from Phase 1 and Phase 2

> This file carries forward only numerical values that are recoverable from the existing Phase 1/2 project history.  
> Missing values are explicitly identified rather than fabricated.

## 1. Baseline Aircraft Model

The Phase 3 controller should continue from the same longitudinal Cessna 182 perturbation model used in Phases 1 and 2.

State vector:

$$x=\begin{bmatrix}u&w&q&\theta\end{bmatrix}^T$$

Linear plant:

$$\dot{x}=Ax+B\delta_e$$

where:

- $u$ = forward-velocity perturbation
- $w$ = vertical body-axis velocity perturbation
- $q$ = pitch-rate perturbation
- $\theta$ = pitch-angle perturbation
- $\delta_e$ = elevator input

The numerical $A$ and $B$ matrices are required for Phase 3 LQR/state-feedback synthesis. Those matrix entries are not present in the retained source excerpts available here, so they are intentionally not reconstructed from poles alone.

---

## 2. Phase 1 Open-Loop Pole Results

The Phase 1 open-loop poles previously obtained were:

$$\lambda_{1,2}=-3.2548\pm1.7168i$$

$$\lambda_{3,4}=-0.0052\pm0.1461i$$

These represent the two dominant longitudinal mode pairs.

### 2.1 Short-Period Mode

Recovered pole pair:

$$\lambda_{SP}=-3.2548\pm1.7168i$$

Natural frequency:

$$\omega_{n,SP}=\sqrt{(-3.2548)^2+(1.7168)^2}=3.679827\ \text{rad/s}$$

Damping ratio:

$$\zeta_{SP}=\frac{3.2548}{3.679827}=0.884498$$

Damped frequency:

$$\omega_{d,SP}=1.7168\ \text{rad/s}$$

Approximate modal time constant:

$$\tau_{SP}=\frac{1}{3.2548}=0.307239\ \text{s}$$

Approximate 2% settling-time estimate:

$$T_{s,SP}\approx\frac{4}{3.2548}=1.228954\ \text{s}$$

### 2.2 Phugoid Mode

Recovered pole pair:

$$\lambda_{PH}=-0.0052\pm0.1461i$$

Natural frequency:

$$\omega_{n,PH}=\sqrt{(-0.0052)^2+(0.1461)^2}=0.146193\ \text{rad/s}$$

Damping ratio:

$$\zeta_{PH}=\frac{0.0052}{0.146193}=0.035570$$

Damped frequency:

$$\omega_{d,PH}=0.1461\ \text{rad/s}$$

Approximate modal time constant:

$$\tau_{PH}=\frac{1}{0.0052}=192.307692\ \text{s}$$

Approximate 2% settling-time estimate:

$$T_{s,PH}\approx\frac{4}{0.0052}=769.230769\ \text{s}$$

### 2.3 Phase 1 Mode Table

| Mode | Pole pair | $\omega_n$ [rad/s] | $\zeta$ | Approx. $T_s$ [s] |
|---|---|---:|---:|---:|
| Short period | $-3.2548\pm1.7168i$ | 3.679827 | 0.884498 | 1.229 |
| Phugoid | $-0.0052\pm0.1461i$ | 0.146193 | 0.035570 | 769.231 |

The phugoid is much slower and much more lightly damped than the short-period mode.

---

## 3. Phase 2 Pitch-Rate Damper

The Phase 2 feedback architecture was

$$\delta_e=-K_q q$$

Using the state selector

$$C_q=\begin{bmatrix}0&0&1&0\end{bmatrix}$$

the feedback law can be written as

$$\delta_e=-K_q C_qx$$

and the closed-loop state matrix becomes

$$A_{cl,q}=A-BK_qC_q$$

for the sign convention above.

### 3.1 Recovered Phase 2 Closed-Loop Pole Pair

The visible Phase 2 result retained in the project history was:

$$\lambda_{SP,cl}=-3.5534\pm2.0426i$$

This gives

$$\omega_{n,SP,cl}=4.098642\ \text{rad/s}$$

and

$$\zeta_{SP,cl}=0.866970$$

with approximate settling-time estimate

$$T_{s,SP,cl}\approx\frac{4}{3.5534}=1.125682\ \text{s}$$

### 3.2 Open-Loop versus Phase 2 Short-Period Comparison

| Quantity | Phase 1 open loop | Phase 2 pitch-rate feedback |
|---|---:|---:|
| Real part magnitude | 3.2548 | 3.5534 |
| Imaginary part | 1.7168 | 2.0426 |
| $\omega_n$ [rad/s] | 3.679827 | 4.098642 |
| $\zeta$ | 0.884498 | 0.866970 |
| Approx. $T_s$ [s] | 1.228954 | 1.125682 |

The recovered numbers show that the short-period poles moved farther into the left half-plane and the natural frequency increased. The damping ratio itself changed from approximately 0.8845 to 0.8670; therefore the Phase 2 result should not be described simply as an increase in damping ratio. A more accurate statement is that the short-period response became faster in terms of exponential decay while its modal frequency also increased.

The second Phase 2 closed-loop pole pair and the numerical value of $K_q$ are not visible in the retained project excerpts, so they are not inserted here.

---

## 4. Phase 3 Starting Point

Phase 3 should not restart from a new aircraft model. It should inherit the same plant:

$$\dot{x}=Ax+B\delta_e$$

and extend the Phase 2 single-state feedback law into full-state feedback:

$$\delta_e=-Kx$$

with

$$K=\begin{bmatrix}k_u&k_w&k_q&k_\theta\end{bmatrix}$$

giving

$$A_{cl}=A-BK$$

This creates the direct progression:

$$\text{Phase 1: open-loop aircraft}\rightarrow\text{Phase 2: }q\text{-feedback}\rightarrow\text{Phase 3: full-state/LQR autopilot}$$

---

## 5. Phase 3 Controllability Calculation

Once the original numerical $A$ and $B$ matrices are restored, calculate

$$\mathcal{C}=\begin{bmatrix}B&AB&A^2B&A^3B\end{bmatrix}$$

and verify

$$\operatorname{rank}(\mathcal{C})=4$$

before performing arbitrary pole placement.

MATLAB:

```matlab
Co = ctrb(A,B);
rank_Co = rank(Co);
n = size(A,1);
```

Do not populate the rank numerically until the original $A$ and $B$ matrices are loaded.

---

## 6. Phase 3 State-Feedback Design

For pole placement:

```matlab
desired_poles = [...];       % selected from Phase 3 requirements
K = place(A,B,desired_poles);
Acl = A - B*K;
p_cl = eig(Acl);
```

The Phase 1 modes provide the baseline against which these new closed-loop poles should be compared.

The controller should not blindly move every pole far left. The final pole locations must balance:

- response speed,
- damping,
- elevator effort,
- robustness,
- actuator bandwidth,
- command-tracking performance.

---

## 7. Phase 3 LQR Design

Phase 3 can use the same plant matrices $A$ and $B$ with

$$J=\int_0^\infty\left(x^TQx+\delta_e^TR\delta_e\right)dt$$

and

$$\delta_e=-K_{LQR}x$$

where

$$K_{LQR}=\operatorname{lqr}(A,B,Q,R)$$

in MATLAB.

Recommended implementation:

```matlab
Q = diag([q_u q_w q_q q_theta]);
R = r_elevator;

[K_lqr,S,p_lqr] = lqr(A,B,Q,R);
Acl_lqr = A - B*K_lqr;
```

The Phase 2 experience with pitch-rate feedback is useful here: $q$ was already identified as an effective feedback state, so $q$ should receive deliberate consideration when selecting the LQR state weighting. The final weight must still be justified from allowable state excursions and actuator effort.

---

## 8. Phase 3 Reference Tracking

For pitch-angle tracking, define

$$C_\theta=\begin{bmatrix}0&0&0&1\end{bmatrix}$$

and use

$$\delta_e=-Kx+N_\theta\theta_c$$

where

$$N_\theta=-\frac{1}{C_\theta(A-BK)^{-1}B}$$

for the SISO configuration.

This converts the regulator into a reference-tracking system.

---

## 9. Altitude State Required for the Autopilot

The Phase 1/2 state vector

$$x=\begin{bmatrix}u&w&q&\theta\end{bmatrix}^T$$

does not contain altitude.

For the Phase 3 altitude autopilot, augment the plant with $h$.

For small perturbations about approximately level flight, using body $z$ positive downward and altitude positive upward,

$$\dot{h}\approx U_0\theta-w$$

so the augmented state is

$$x_h=\begin{bmatrix}u&w&q&\theta&h\end{bmatrix}^T$$

and

$$A_h=
\begin{bmatrix}
A & 0\\
0 & -1 & 0 & U_0 & 0
\end{bmatrix}$$

with the final row interpreted as the altitude kinematic equation above.

The original Phase 1 trim speed $U_0$ is required before this equation can be populated numerically.

---

## 10. Values Already Available for Phase 3

The following numerical baseline can already be inserted into `phase3_results.md`:

| Phase 3 baseline quantity | Value |
|---|---|
| State vector | $[u,w,q,\theta]^T$ |
| Open-loop short-period poles | $-3.2548\pm1.7168i$ |
| Open-loop short-period $\omega_n$ | 3.679827 rad/s |
| Open-loop short-period $\zeta$ | 0.884498 |
| Open-loop phugoid poles | $-0.0052\pm0.1461i$ |
| Open-loop phugoid $\omega_n$ | 0.146193 rad/s |
| Open-loop phugoid $\zeta$ | 0.035570 |
| Phase 2 visible closed-loop short-period poles | $-3.5534\pm2.0426i$ |
| Phase 2 short-period $\omega_n$ | 4.098642 rad/s |
| Phase 2 short-period $\zeta$ | 0.866970 |
| Phase 1 control input | Elevator $\delta_e$ |
| Phase 2 control law | $\delta_e=-K_q q$ |
| Phase 3 state-feedback law | $\delta_e=-Kx$ |
| Phase 3 LQR law | $\delta_e=-K_{LQR}x$ |

---

## 11. Numerical Values Still Needed from the Original Phase 1/2 Files

The following cannot be reconstructed reliably from pole locations alone:

1. Numerical $A$ matrix
2. Numerical $B$ matrix
3. Trim airspeed $U_0$
4. Trim altitude $h_0$
5. Phase 2 $K_q$
6. Complete Phase 2 closed-loop pole set
7. Elevator physical/simulation limits
8. Final $Q$ and $R$ design choices

Once the original $A$ and $B$ matrices are restored, all of the remaining Phase 3 quantities—controllability, pole-placement gain, LQR gain, pitch tracking, altitude tracking, control effort, and disturbance rejection—can be generated directly and reproducibly.
