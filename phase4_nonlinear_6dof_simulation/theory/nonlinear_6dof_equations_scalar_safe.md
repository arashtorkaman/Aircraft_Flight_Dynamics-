# Nonlinear 6-DOF Aircraft Equations of Motion

## Project Context

This document defines the nonlinear six-degree-of-freedom equations of motion used for an F-22-inspired MIMO fighter-aircraft simulation.

The model is intended for educational flight-dynamics, control-design, and simulation purposes. It is not a representation of the actual F-22 flight-control system, proprietary aerodynamic database, or classified aircraft model.

---

## 1. Purpose of the 6-DOF Model

A six-degree-of-freedom aircraft model captures coupled nonlinear motion in three translational and three rotational axes.

The model is used for:

- open-loop aircraft response simulation,
- trim calculation,
- numerical linearization,
- MIMO state-space control design,
- LQR closed-loop simulation,
- trajectory visualization,
- future automatic landing and guidance-law development.

---

## 2. Coordinate Frames

### 2.1 Body Frame

The body frame is fixed to the aircraft center of gravity.

- The positive body $x_b$-axis points forward through the aircraft nose.
- The positive body $y_b$-axis points out the right wing.
- The positive body $z_b$-axis points downward.

The body-axis velocity vector is:

$$
V_b = [u, v, w]^T
$$

The body-axis angular-rate vector is:

$$
\omega_b = [p, q, r]^T
$$

where:

- $u$ is forward body-axis velocity,
- $v$ is lateral body-axis velocity,
- $w$ is vertical body-axis velocity,
- $p$ is roll rate,
- $q$ is pitch rate,
- $r$ is yaw rate.

### 2.2 Earth Frame

The Earth frame is assumed to be locally flat and inertial for this simulation.

The Earth-frame position vector is:

$$
r_e = [X_E, Y_E, Z_E]^T
$$

where:

- $X_E$ is forward/inertial position,
- $Y_E$ is lateral/inertial position,
- $Z_E$ is vertical/downward position.

With the positive $Z_E$ direction downward, altitude is approximately:

$$
h = -Z_E
$$

---

## 3. Full Nonlinear State Vector

The nonlinear simulation state vector is:

$$
x = [u, v, w, p, q, r, \phi, \theta, \psi, X_E, Y_E, Z_E]^T
$$

| State | Unit | Description |
|---|---:|---|
| $u$ | m/s | Forward body-axis velocity |
| $v$ | m/s | Lateral body-axis velocity |
| $w$ | m/s | Vertical body-axis velocity |
| $p$ | rad/s | Roll rate |
| $q$ | rad/s | Pitch rate |
| $r$ | rad/s | Yaw rate |
| $\phi$ | rad | Roll angle |
| $\theta$ | rad | Pitch angle |
| $\psi$ | rad | Yaw/heading angle |
| $X_E$ | m | Earth-frame forward position |
| $Y_E$ | m | Earth-frame lateral position |
| $Z_E$ | m | Earth-frame vertical/down position |

---

## 4. Control Input Vector

For the F-22-inspired MIMO fighter model, the control input vector is:

$$
u_c = [\delta_e, \delta_a, \delta_r, \delta_T, \delta_{tv}]^T
$$

| Input | Unit | Description |
|---|---:|---|
| $\delta_e$ | rad | Elevator/stabilator command |
| $\delta_a$ | rad | Aileron command |
| $\delta_r$ | rad | Rudder command |
| $\delta_T$ | nondimensional | Normalized throttle command |
| $\delta_{tv}$ | rad | Pitch thrust-vectoring command |

The first implementation treats these as idealized actuator commands. Later versions may add actuator dynamics, rate limits, dead zones, saturation, and control allocation logic.

---

## 5. Airspeed, Angle of Attack, and Sideslip

The total airspeed is:

$$
V = \sqrt{u^2 + v^2 + w^2}
$$

The angle of attack is:

$$
\alpha = \tan^{-1}\left(\frac{w}{u}\right)
$$

In implementation, use `atan2(w,u)` for numerical robustness.

The sideslip angle is:

$$
\beta = \sin^{-1}\left(\frac{v}{V}\right)
$$

For numerical robustness, the implementation should prevent division by zero when $V$ is very small.

```matlab
V = sqrt(u^2 + v^2 + w^2);
V = max(V, 1e-3);

alpha = atan2(w, u);
beta  = asin(max(min(v/V, 1), -1));
```

---

## 6. Dynamic Pressure

Dynamic pressure is defined as:

$$
\bar{q} = \frac{1}{2}\rho V^2
$$

where:

- $\rho$ is air density,
- $V$ is true airspeed.

In the first simulation version, constant sea-level density may be used:

$$
\rho = 1.225 \; \text{kg/m}^3
$$

A later version can replace this with a standard-atmosphere model:

$$
\rho = \rho(h)
$$

---

## 7. Nondimensional Angular Rates

The aerodynamic damping terms use nondimensional angular rates.

Roll-rate term:

$$
\hat{p} = \frac{pb}{2V}
$$

Pitch-rate term:

$$
\hat{q} = \frac{q\bar{c}}{2V}
$$

Yaw-rate term:

$$
\hat{r} = \frac{rb}{2V}
$$

where:

- $b$ is wingspan,
- $\bar{c}$ is mean aerodynamic chord,
- $V$ is airspeed.

---

## 8. Aerodynamic Force and Moment Coefficients

The aerodynamic model is written in coefficient form.

The body-axis force coefficients are:

$$
C_X, \quad C_Y, \quad C_Z
$$

The body-axis moment coefficients are:

$$
C_l, \quad C_m, \quad C_n
$$

A simplified coefficient model may be written as:

$$
C_X = C_{X0} + C_{X\alpha}\alpha + C_{X\delta_e}\delta_e
$$

$$
C_Y = C_{Y\beta}\beta + C_{Y\hat{p}}\hat{p} + C_{Y\hat{r}}\hat{r} + C_{Y\delta_a}\delta_a + C_{Y\delta_r}\delta_r
$$

$$
C_Z = C_{Z0} + C_{Z\alpha}\alpha + C_{Z\hat{q}}\hat{q} + C_{Z\delta_e}\delta_e
$$

$$
C_l = C_{l\beta}\beta + C_{l\hat{p}}\hat{p} + C_{l\hat{r}}\hat{r} + C_{l\delta_a}\delta_a + C_{l\delta_r}\delta_r
$$

$$
C_m = C_{m0} + C_{m\alpha}\alpha + C_{m\hat{q}}\hat{q} + C_{m\delta_e}\delta_e + C_{m\delta_{tv}}\delta_{tv}
$$

$$
C_n = C_{n\beta}\beta + C_{n\hat{p}}\hat{p} + C_{n\hat{r}}\hat{r} + C_{n\delta_a}\delta_a + C_{n\delta_r}\delta_r
$$

The signs of the main stability derivatives should be chosen to produce physically reasonable aircraft behavior:

| Stability derivative | Required sign | Physical meaning |
|---|---:|---|
| $C_{m\alpha}$ | $< 0$ | Longitudinal static stability in the simplified model |
| $C_{m\hat{q}}$ | $< 0$ | Pitch-rate damping |
| $C_{l\hat{p}}$ | $< 0$ | Roll-rate damping |
| $C_{n\hat{r}}$ | $< 0$ | Yaw-rate damping |
| $C_{n\beta}$ | $> 0$ | Directional weathercock stability |

---

## 9. Aerodynamic Forces and Moments

The aerodynamic body-axis forces are computed from the force coefficients:

$$
X_A = \bar{q}SC_X
$$

$$
Y_A = \bar{q}SC_Y
$$

$$
Z_A = \bar{q}SC_Z
$$

The aerodynamic body-axis moments are:

$$
L_A = \bar{q}SbC_l
$$

$$
M_A = \bar{q}S\bar{c}C_m
$$

$$
N_A = \bar{q}SbC_n
$$

where:

- $S$ is wing reference area,
- $b$ is wingspan,
- $\bar{c}$ is mean aerodynamic chord.

---

## 10. Propulsion and Thrust Vectoring

The total engine thrust is modeled as:

$$
T = \delta_T T_{max}
$$

where:

- $\delta_T$ is normalized throttle command,
- $T_{max}$ is maximum available thrust.

For a simplified thrust-vectoring model, the thrust components may be approximated as:

$$
X_T = T\cos(\delta_{tv})
$$

$$
Z_T = -T\sin(\delta_{tv})
$$

The thrust-vectoring pitching moment may be approximated as:

$$
M_T = Z_T l_T
$$

where $l_T$ is the moment arm from the center of gravity to the thrust-vectoring force application point.

The total body-axis forces become:

$$
X = X_A + X_T
$$

$$
Y = Y_A
$$

$$
Z = Z_A + Z_T
$$

The total moments become:

$$
L = L_A
$$

$$
M = M_A + M_T
$$

$$
N = N_A
$$

A more advanced version may include asymmetric thrust-vectoring effects, engine gyroscopic moments, inlet effects, and nozzle dynamics.

---

## 11. Translational Equations of Motion

The nonlinear body-axis translational equations are:

$$
\dot{u} = rv - qw + \frac{X}{m} - g\sin\theta
$$

$$
\dot{v} = pw - ru + \frac{Y}{m} + g\sin\phi\cos\theta
$$

$$
\dot{w} = qu - pv + \frac{Z}{m} + g\cos\phi\cos\theta
$$

where:

- $X$, $Y$, and $Z$ are total body-axis forces,
- $m$ is aircraft mass,
- $g$ is gravitational acceleration.

These equations are nonlinear because of:

- products of angular rates and velocities,
- trigonometric gravity terms,
- aerodynamic coefficients depending on $\alpha$, $\beta$, $\hat{p}$, $\hat{q}$, and $\hat{r}$,
- dynamic pressure dependence on $V^2$.

---

## 12. Rotational Equations of Motion

For an aircraft with a nonzero product of inertia $I_{xz}$, the rotational equations are:

$$
I_x\dot{p} - I_{xz}\dot{r} = L + I_{xz}pq - (I_z - I_y)qr
$$

$$
I_y\dot{q} = M + (I_z - I_x)pr - I_{xz}(p^2-r^2)
$$

$$
I_z\dot{r} - I_{xz}\dot{p} = N + (I_x - I_y)pq - I_{xz}qr
$$

Define:

$$
A_p = L + I_{xz}pq - (I_z - I_y)qr
$$

$$
A_r = N + (I_x - I_y)pq - I_{xz}qr
$$

The coupled roll-yaw equations are:

$$
I_x\dot{p} - I_{xz}\dot{r} = A_p
$$

$$
-I_{xz}\dot{p} + I_z\dot{r} = A_r
$$

Solving the coupled two-equation system gives:

$$
\dot{p} = \frac{I_z A_p + I_{xz} A_r}{I_x I_z - I_{xz}^2}
$$

$$
\dot{r} = \frac{I_{xz} A_p + I_x A_r}{I_x I_z - I_{xz}^2}
$$

The pitch-rate equation is:

$$
\dot{q} = \frac{1}{I_y}\left[M + (I_z - I_x)pr - I_{xz}(p^2-r^2)\right]
$$

If the product of inertia is neglected, $I_{xz}=0$, the rotational equations simplify to:

$$
\dot{p} = \frac{L - (I_z - I_y)qr}{I_x}
$$

$$
\dot{q} = \frac{M + (I_z - I_x)pr}{I_y}
$$

$$
\dot{r} = \frac{N + (I_x - I_y)pq}{I_z}
$$

---

## 13. Euler Angle Kinematics

The Euler angle kinematic equations are:

$$
\dot{\phi} = p + q\sin\phi\tan\theta + r\cos\phi\tan\theta
$$

$$
\dot{\theta} = q\cos\phi - r\sin\phi
$$

$$
\dot{\psi} = \frac{q\sin\phi + r\cos\phi}{\cos\theta}
$$

These equations map body-axis angular rates into Euler angle rates.

A limitation of Euler angles is the singularity at:

$$
\theta = \pm 90^\circ
$$

For the intended flight regimes in this project, Euler angles are acceptable. For aggressive post-stall maneuvering or high-angle-of-attack simulation, quaternions would be preferable.

---

## 14. Earth-Frame Position Kinematics

The Earth-frame velocity is obtained by rotating body-axis velocity into the Earth frame:

$$
\dot{X}_E = u\cos\theta\cos\psi + v(\sin\phi\sin\theta\cos\psi - \cos\phi\sin\psi) + w(\cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi)
$$

$$
\dot{Y}_E = u\cos\theta\sin\psi + v(\sin\phi\sin\theta\sin\psi + \cos\phi\cos\psi) + w(\cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi)
$$

$$
\dot{Z}_E = -u\sin\theta + v\sin\phi\cos\theta + w\cos\phi\cos\theta
$$

With the positive $Z_E$ direction downward, altitude is:

$$
h = -Z_E
$$

and altitude rate is:

$$
\dot{h} = -\dot{Z}_E
$$

---

## 15. Complete Nonlinear State Derivative

The nonlinear state derivative is:

$$
\dot{x} = f(x,u_c)
$$

where:

$$
\dot{x} = [\dot{u}, \dot{v}, \dot{w}, \dot{p}, \dot{q}, \dot{r}, \dot{\phi}, \dot{\theta}, \dot{\psi}, \dot{X}_E, \dot{Y}_E, \dot{Z}_E]^T
$$

and:

$$
f(x,u_c) = [f_u, f_v, f_w, f_p, f_q, f_r, f_\phi, f_\theta, f_\psi, f_X, f_Y, f_Z]^T
$$

This notation represents the complete nonlinear 6-DOF aircraft model.

---

## 16. Numerical Linearization

For MIMO control design, the nonlinear model is linearized around a trim condition.

Let the trim state and trim input be:

$$
x_0, \quad u_{c0}
$$

Small perturbations are defined as:

$$
\Delta x = x - x_0
$$

$$
\Delta u_c = u_c - u_{c0}
$$

The linearized model is:

$$
\Delta \dot{x} = A\Delta x + B\Delta u_c
$$

where:

$$
A = \left.\frac{\partial f}{\partial x}\right|_{x_0,u_{c0}}
$$

$$
B = \left.\frac{\partial f}{\partial u_c}\right|_{x_0,u_{c0}}
$$

For numerical implementation, each column of $A$ and $B$ can be estimated using central finite differences.

---

## 17. MIMO Control Interpretation

The nonlinear 6-DOF aircraft model contains coupled longitudinal, lateral, and directional dynamics.

The control input vector contains multiple actuators:

$$
u_c = [\delta_e, \delta_a, \delta_r, \delta_T, \delta_{tv}]^T
$$

A MIMO controller coordinates these inputs to regulate multiple aircraft states at the same time.

For an LQR controller, the linearized system is:

$$
\Delta\dot{x} = A\Delta x + B\Delta u_c
$$

The feedback law is:

$$
\Delta u_c = -K\Delta x
$$

where $K$ is the MIMO gain matrix computed from the LQR design.

---

## 18. Modeling Limitations

This model is simplified and intended for educational use. The following limitations apply:

- aerodynamic coefficients are simplified,
- no real F-22 aerodynamic database is used,
- actuator dynamics are initially neglected,
- structural flexibility is neglected,
- engine dynamics are simplified,
- Earth curvature is neglected,
- atmospheric density may be constant in the first implementation,
- Euler angles are used instead of quaternions.

These assumptions are acceptable for a portfolio-level flight-dynamics and control project, provided they are clearly documented.

---

## 19. Recommended Validation Plots

The implementation should generate:

- open-loop body-axis velocity response,
- angular-rate response,
- Euler-angle response,
- Earth-frame trajectory,
- control input histories,
- trim residual check,
- linear versus nonlinear response comparison,
- open-loop versus closed-loop eigenvalue plot.

---

## 20. Summary

This document defines the nonlinear 6-DOF equations of motion used as the plant model for the F-22-inspired MIMO fighter aircraft simulation.

The model supports:

- nonlinear rigid-body aircraft simulation,
- aerodynamic force and moment modeling,
- trim calculation,
- numerical linearization,
- MIMO LQR controller design,
- future automatic landing and guidance-law development.

The key purpose of this model is not to reproduce the real F-22, but to demonstrate professional aerospace flight-dynamics and control-system engineering methods.
