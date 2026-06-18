# Modeling Assumptions

## Project Context

This document defines the modeling assumptions used in the **F-22-inspired MIMO 6-DOF fighter aircraft simulation**. The project is intended for educational aerospace controls, flight dynamics, and portfolio demonstration purposes.

The model is **not** a real F-22 flight-control model. It does not use proprietary aerodynamic tables, classified control laws, verified mass properties, or actual gain-scheduled flight-control software. Instead, it uses a public-data-inspired aircraft configuration and a simplified nonlinear rigid-body model to demonstrate the engineering workflow used in flight dynamics and control design.

The main objective is to show the process of building:

- a nonlinear 6-DOF aircraft model,
- a representative aerodynamic force and moment model,
- a trim and linearization workflow,
- a multivariable state-space representation,
- and a MIMO LQR controller for attitude and rate stabilization.

---

## 1. Aircraft Representation Assumptions

The aircraft is modeled as a **generic high-performance fighter inspired by the F-22 configuration**.

The following assumptions are used:

- The aircraft is treated as a rigid body.
- The aircraft mass is constant during the simulation.
- Fuel burn is neglected.
- Store separation, weapon deployment, landing gear motion, and payload changes are neglected.
- Structural flexibility is neglected.
- Aeroelastic effects are neglected.
- Engine gyroscopic effects are neglected.
- The aircraft is assumed to be symmetric about the longitudinal plane.
- Inertia cross-products are neglected except for $I_{xz}$, which may be retained for fighter-like coupling.

The model therefore represents a **rigid-body flight dynamics approximation**, not a complete high-fidelity fighter simulation.

---

## 2. Coordinate System Assumptions

The body-axis coordinate system is defined as:

- $x_b$: positive forward through the nose,
- $y_b$: positive out the right wing,
- $z_b$: positive downward.

The Earth/inertial frame is assumed to be a local flat-Earth frame:

- $X_E$: horizontal position along the initial flight direction,
- $Y_E$: lateral position,
- $Z_E$: positive downward altitude coordinate.

The simulation uses Euler angles:

$$
\phi = \text{roll angle}
$$

$$
\theta = \text{pitch angle}
$$

$$
\psi = \text{yaw angle}
$$

Euler angles are used because they are intuitive for aircraft stability and control analysis. However, this introduces a known singularity when $\theta \rightarrow \pm 90^\circ$.

For the current project, the aircraft is not simulated near vertical attitude, so Euler-angle singularities are not expected to affect the results.

---

## 3. Flight Regime Assumptions

The model is intended for moderate-angle flight simulation around a representative fighter trim condition.

The first implementation assumes:

- subsonic or low-transonic operation,
- moderate angle of attack,
- moderate sideslip,
- no post-stall aerodynamics,
- no departure or spin modeling,
- no shock effects,
- no compressibility correction in the first version,
- no high-angle-of-attack vortex-lift model.

The model can be used to demonstrate nonlinear rigid-body dynamics, but it should not be used to predict real fighter performance at high angle of attack, supersonic speed, or post-stall maneuvering.

---

## 4. Atmosphere Assumptions

The first version of the simulation assumes a simple atmosphere model.

The default assumptions are:

- constant air density $\rho$ for low-altitude or short-duration simulation,
- constant gravitational acceleration $g = 9.80665 \text{ m/s}^2$,
- no wind,
- no turbulence,
- no gust model,
- no atmospheric temperature variation,
- no pressure-altitude variation.

Dynamic pressure is computed as:

$$
\bar{q} = \frac{1}{2}\rho V^2
$$

where:

$$
V = \sqrt{u^2 + v^2 + w^2}
$$

For a future version, the constant-density model can be replaced by the International Standard Atmosphere or MATLAB Aerospace Blockset atmosphere blocks.

---

## 5. Aerodynamic Model Assumptions

The aerodynamic model is coefficient-based. The aerodynamic forces and moments are computed from nondimensional coefficients:

$$
C_X, C_Y, C_Z, C_l, C_m, C_n
$$

The aerodynamic force model is:

$$
X = \bar{q}S C_X + T_x
$$

$$
Y = \bar{q}S C_Y + T_y
$$

$$
Z = \bar{q}S C_Z + T_z
$$

The aerodynamic moment model is:

$$
L = \bar{q}SbC_l
$$

$$
M = \bar{q}S\bar{c}C_m
$$

$$
N = \bar{q}SbC_n
$$

The first implementation assumes linear or weakly nonlinear aerodynamic derivatives around a trim condition. Example coefficient structure:

$$
C_m = C_{m0} + C_{m_{\alpha}}\alpha + C_{m_{\hat{q}}}\hat{q} + C_{m_{\delta_e}}\delta_e + C_{m_{\delta_{tv}}}\delta_{tv}
$$

$$
C_l = C_{l_{\beta}}\beta + C_{l_{\hat{p}}}\hat{p} + C_{l_{\hat{r}}}\hat{r} + C_{l_{\delta_a}}\delta_a + C_{l_{\delta_r}}\delta_r
$$

$$
C_n = C_{n_{\beta}}\beta + C_{n_{\hat{p}}}\hat{p} + C_{n_{\hat{r}}}\hat{r} + C_{n_{\delta_a}}\delta_a + C_{n_{\delta_r}}\delta_r
$$

The nondimensional angular rates are:

$$
\hat{p} = \frac{pb}{2V}
$$

$$
\hat{q} = \frac{q\bar{c}}{2V}
$$

$$
\hat{r} = \frac{rb}{2V}
$$

The signs of the main stability derivatives are selected to produce physically reasonable aircraft behavior:

| Stability derivative | Required sign | Physical meaning |
|---|---:|---|
| $C_{m_{\alpha}}$ | $< 0$ | Longitudinal static stability in the simplified model |
| $C_{m_{\hat{q}}}$ | $< 0$ | Pitch-rate damping |
| $C_{l_{\hat{p}}}$ | $< 0$ | Roll-rate damping |
| $C_{n_{\hat{r}}}$ | $< 0$ | Yaw-rate damping |
| $C_{n_{\beta}}$ | $> 0$ | Directional weathercock stability |

These coefficients are representative educational values and are not real F-22 aerodynamic derivatives.

---

## 6. Control Surface Assumptions

The aircraft control inputs are modeled as:

$$
u_c =
\begin{bmatrix}
\delta_e \\
\delta_a \\
\delta_r \\
\delta_T \\
\delta_{tv}
\end{bmatrix}
$$

where:

- $\delta_e$ is elevator or stabilator deflection,
- $\delta_a$ is aileron deflection,
- $\delta_r$ is rudder deflection,
- $\delta_T$ is normalized throttle command,
- $\delta_{tv}$ is pitch-axis thrust-vectoring command.

The first implementation assumes:

- symmetric stabilator behavior is represented by a single elevator-equivalent input,
- differential stabilator effects are not modeled separately,
- leading-edge flaps are neglected,
- trailing-edge flaps are neglected,
- control-surface actuator dynamics are neglected in the first version,
- control deflections are limited by saturation blocks,
- rate limits are optional and may be added later.

Typical simplified actuator limits are:

| Input | Limit |
|---|---:|
| Elevator/stabilator, $\delta_e$ | $\pm 25^\circ$ |
| Aileron, $\delta_a$ | $\pm 25^\circ$ |
| Rudder, $\delta_r$ | $\pm 30^\circ$ |
| Throttle perturbation, $\delta_T$ | $0 \leq \delta_T \leq 1$ |
| Thrust-vectoring pitch angle, $\delta_{tv}$ | $\pm 20^\circ$ |

These limits are representative and are not claimed to match the actual F-22 actuation system.

---

## 7. Propulsion and Thrust-Vectoring Assumptions

The propulsion model is simplified.

The first implementation assumes:

- total thrust is commanded through a normalized throttle input,
- throttle response is instantaneous,
- engine spool dynamics are neglected,
- thrust lapse with altitude and Mach number is neglected,
- inlet distortion is neglected,
- engine limits are simplified,
- thrust vectoring is modeled as a pitch-plane force-vector rotation.

The thrust force can be approximated as:

$$
T = \delta_T T_{max}
$$

If pitch thrust vectoring is included, the body-axis thrust components are approximated as:

$$
T_x = T\cos(\delta_{tv})
$$

$$
T_z = -T\sin(\delta_{tv})
$$

The thrust-vectoring moment can be represented either by an equivalent pitching-moment coefficient or by a moment arm model:

$$
M_{tv} = l_{tv}T_z
$$

where $l_{tv}$ is the distance from the center of gravity to the thrust-vectoring force application point.

---

## 8. Mass and Inertia Assumptions

The model uses representative mass and inertia values.

The first implementation assumes:

- mass is constant,
- center of gravity is fixed,
- fuel slosh is neglected,
- weapon bay effects are neglected,
- external stores are neglected,
- landing gear effects are neglected,
- inertia tensor is constant.

The inertia matrix is approximated as:

$$
I =
\begin{bmatrix}
I_x & 0 & -I_{xz} \\
0 & I_y & 0 \\
-I_{xz} & 0 & I_z
\end{bmatrix}
$$

For early simulations, $I_{xz}$ may be set to zero to simplify debugging. Once the basic model is working, $I_{xz}$ can be included to demonstrate fighter-like inertial coupling.

---

## 9. Trim Assumptions

The trim condition is assumed to represent steady, wings-level flight.

The trim target is:

$$
\dot{x} = 0
$$

for the dynamic states:

$$
\begin{bmatrix}
u & v & w & p & q & r & \phi & \theta & \psi
\end{bmatrix}^T
$$

A simplified level-flight trim condition assumes:

- $v = 0$,
- $p = q = r = 0$,
- $\phi = 0$,
- $\beta = 0$,
- $\theta \approx \alpha$,
- lift approximately balances weight,
- thrust approximately balances drag.

The approximate trim force balance is:

$$
L \approx W
$$

$$
T \approx D
$$

A numerical optimizer may later be used to solve for:

$$
\alpha_{trim}, \quad \delta_{e,trim}, \quad \delta_{T,trim}
$$

such that the aircraft satisfies the desired steady-flight condition.

---

## 10. Linearization Assumptions

The nonlinear model is linearized around a trim point using numerical finite differences.

The local perturbation model is:

$$
\Delta \dot{x} = A\Delta x + B\Delta u
$$

where:

$$
A = \frac{\partial f}{\partial x}\bigg|_{x_{trim},u_{trim}}
$$

$$
B = \frac{\partial f}{\partial u}\bigg|_{x_{trim},u_{trim}}
$$

The linearized model is valid only near the selected trim point. It should not be interpreted as globally valid over the full flight envelope.

The first MIMO controller uses the dynamic-state subset:

$$
\Delta x_c =
\begin{bmatrix}
\Delta u \\
\Delta v \\
\Delta w \\
\Delta p \\
\Delta q \\
\Delta r \\
\Delta \phi \\
\Delta \theta \\
\Delta \psi
\end{bmatrix}
$$

The position states $X_E$, $Y_E$, and $Z_E$ are used for trajectory plotting but are not included in the first attitude-stabilization LQR design.

---

## 11. MIMO LQR Control Assumptions

The MIMO LQR controller is designed using the linearized aircraft model.

The control law is:

$$
\Delta u_c = -K\Delta x_c
$$

The full command is:

$$
u_c = u_{trim} + \Delta u_c
$$

The LQR cost function is:

$$
J = \int_0^\infty \left(\Delta x^TQ\Delta x + \Delta u^TR\Delta u\right)dt
$$

The weighting matrices $Q$ and $R$ are initially selected using Bryson's Rule:

$$
Q_{ii} = \frac{1}{x_{i,max}^2}
$$

$$
R_{jj} = \frac{1}{u_{j,max}^2}
$$

This gives the weights a physical interpretation based on maximum acceptable state deviations and actuator deflections.

The controller is assumed to have full-state feedback in the first version. Sensor noise, estimator dynamics, Kalman filtering, and actuator delays are not included initially.

---

## 12. Simulation Assumptions

The simulation is performed using numerical integration of the nonlinear equations of motion.

The first implementation assumes:

- continuous-time dynamics,
- fixed or adaptive numerical integration,
- no sensor noise,
- no state-estimation delay,
- no actuator delay,
- no digital sampling effects,
- no flight-control computer latency,
- no actuator structural flexibility,
- no failure modes.

Future versions can add:

- discrete-time implementation,
- sensor models,
- actuator dynamics,
- rate limiters,
- control allocation,
- gain scheduling,
- gust response,
- Monte Carlo uncertainty analysis.

---

## 13. Validation Assumptions

Because the model does not use real F-22 aerodynamic databases, validation is based on engineering consistency rather than real-aircraft matching.

The model is considered acceptable for this portfolio stage if:

- forces and moments have physically reasonable signs,
- trim produces approximately steady flight,
- open-loop modes are plausible for a fighter-like aircraft,
- closed-loop poles move to stable locations under LQR control,
- control inputs remain within saturation limits,
- attitude responses are bounded and well damped,
- the aircraft trajectory is physically reasonable,
- no state diverges unexpectedly during nominal closed-loop tests.

The model is not validated against flight-test data and should not be used for design, certification, safety analysis, or operational prediction.

---

## 14. Known Limitations

The current model has the following limitations:

- It is not an actual F-22 model.
- Aerodynamic coefficients are representative, not measured.
- High-angle-of-attack aerodynamics are not modeled.
- Supersonic effects are not modeled.
- Compressibility effects are not modeled in the first version.
- Full control allocation is not modeled.
- Real flight-control laws are not represented.
- Real actuator dynamics are not represented.
- Real sensor and estimator systems are not represented.
- Engine dynamics are simplified.
- Aeroelastic effects are neglected.
- Structural limits are not modeled.
- Pilot-command shaping is not modeled.

These limitations should be stated clearly in the README and documentation.

---

## 15. Future Improvements

The following improvements can be added in later phases:

1. Replace constant-density atmosphere with an altitude-dependent atmosphere model.
2. Add Mach-dependent aerodynamic coefficients.
3. Add nonlinear lift and moment behavior at high angle of attack.
4. Add actuator first-order dynamics.
5. Add actuator rate limits.
6. Add sensor noise and bias models.
7. Add Kalman filtering for state estimation.
8. Add discrete-time flight-control implementation.
9. Add gain scheduling across airspeed and altitude.
10. Add control allocation for multiple redundant effectors.
11. Add turbulence and gust models.
12. Add Monte Carlo uncertainty analysis.
13. Add automatic landing or approach-mode guidance.
14. Add Simulink implementation using Aerospace Blockset.

---

## Summary

This model is designed to demonstrate the engineering workflow behind nonlinear aircraft simulation and MIMO control design. The assumptions intentionally simplify the real aircraft so the project remains transparent, implementable, and suitable for a portfolio.

The most important assumption is that this is an **F-22-inspired educational fighter model**, not a real F-22 simulation. The value of the project comes from the modeling, control, simulation, documentation, and validation workflow rather than from claiming exact aircraft fidelity.
