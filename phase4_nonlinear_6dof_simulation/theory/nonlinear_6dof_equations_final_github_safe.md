# Nonlinear 6-DOF Aircraft Equations of Motion

## Project Context

This document defines the nonlinear six-degree-of-freedom equations of motion used for an **F-22-inspired MIMO fighter aircraft simulation**. The model describes rigid-body translational motion, rotational motion, attitude kinematics, and Earth-frame position dynamics using body-axis velocities, angular rates, Euler angles, and inertial position states.

This model is intended for educational flight-dynamics, control-design, and simulation purposes. It is **not** a representation of the actual F-22 flight-control system, proprietary aerodynamic database, or classified aircraft model.

---

## 1. Purpose of the 6-DOF Model

A six-degree-of-freedom aircraft model captures the coupled nonlinear motion of the aircraft in three translational and three rotational axes.

| Degree of freedom | Motion type | Variable examples |
|---|---|---|
| Longitudinal translation | Forward/backward body-axis motion | $u$ |
| Lateral translation | Side-to-side body-axis motion | $v$ |
| Vertical translation | Up/down body-axis motion | $w$ |
| Roll rotation | Rotation about body $x$-axis | $p$, $\phi$ |
| Pitch rotation | Rotation about body $y$-axis | $q$, $\theta$ |
| Yaw rotation | Rotation about body $z$-axis | $r$, $\psi$ |

The nonlinear 6-DOF model is used as the simulation plant for:

- open-loop aircraft response,
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

| Axis | Direction |
|---|---|
| $x_b$ | Positive forward through the aircraft nose |
| $y_b$ | Positive out the right wing |
| $z_b$ | Positive downward |

The body-axis velocity vector is:

$$
V_b
=
\begin{bmatrix}
u \\
v \\
w
\end{bmatrix}
$$

where:

- $u$ is forward body-axis velocity,
- $v$ is lateral body-axis velocity,
- $w$ is vertical body-axis velocity.

The body-axis angular-rate vector is:

$$
\omega_b
=
\begin{bmatrix}
p \\
q \\
r
\end{bmatrix}
$$

where:

- $p$ is roll rate,
- $q$ is pitch rate,
- $r$ is yaw rate.

### 2.2 Earth Frame

The Earth frame is assumed to be locally flat and inertial for this simulation.

The Earth-frame position vector is:

$$
r_e
=
\begin{bmatrix}
X_E \\
Y_E \\
Z_E
\end{bmatrix}
$$

where:

- $X_E$ is forward/inertial position,
- $Y_E$ is lateral/inertial position,
- $Z_E$ is vertical/downward position.

With the downward-positive Earth-frame convention, altitude is approximately:

$$
h = -Z_E
$$

---

## 3. Full Nonlinear State Vector

The nonlinear simulation state vector is:

$$
x
=
\begin{bmatrix}
u \\
v \\
w \\
p \\
q \\
r \\
\phi \\
\theta \\
\psi \\
X_E \\
Y_E \\
Z_E
\end{bmatrix}
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
u_c
=
\begin{bmatrix}
\delta_e \\
\delta_a \\
\delta_r \\
\delta_T \\
\delta_{tv}
\end{bmatrix}
$$

| Input | Unit | Description |
|---|---:|---|
| $\delta_e$ | rad | Elevator/stabilator command |
| $\delta_a$ | rad | Aileron command |
| $\delta_r$ | rad | Rudder command |
| $\delta_T$ | nondimensional | Normalized throttle command |
| $\delta_{tv}$ | rad | Pitch thrust-vectoring command |

The first implementation treats these as idealized actuator commands. Later versions may add actuator dynamics, rate limits, dead zones, saturation, and control-allocation logic.

---

## 5. Airspeed, Angle of Attack, and Sideslip

The total airspeed is:

$$
V
=
\sqrt{u^2 + v^2 + w^2}
$$

The angle of attack is:

$$
\alpha
=
\tan^{-1}\left(\frac{w}{u}\right)
$$

In software, the more robust implementation is:

$$
\alpha
=
\operatorname{atan2}(w,u)
$$

The sideslip angle is:

$$
\beta
=
\sin^{-1}\left(\frac{v}{V}\right)
$$

For numerical robustness, the implementation should prevent division by zero when $V$ is very small.

A practical MATLAB implementation is:

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
\bar{q}
=
\frac{1}{2}\rho V^2
$$

where:

- $\rho$ is air density,
- $V$ is true airspeed.

In the first simulation version, constant sea-level air density may be used:

$$
\rho = 1.225 \ \text{kg/m}^3
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
\hat{p}
=
\frac{pb}{2V}
$$

Pitch-rate term:

$$
\hat{q}
=
\frac{q\bar{c}}{2V}
$$

Yaw-rate term:

$$
\hat{r}
=
\frac{rb}{2V}
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
C_X
=
C_{X0}
+
C_{X\alpha}\alpha
+
C_{X\delta_e}\delta_e
$$

$$
C_Y
=
C_{Y\beta}\beta
+
C_{Y\hat{p}}\hat{p}
+
C_{Y\hat{r}}\hat{r}
+
C_{Y\delta_a}\delta_a
+
C_{Y\delta_r}\delta_r
$$

$$
C_Z
=
C_{Z0}
+
C_{Z\alpha}\alpha
+
C_{Z\hat{q}}\hat{q}
+
C_{Z\delta_e}\delta_e
$$

$$
C_l
=
C_{l\beta}\beta
+
C_{l\hat{p}}\hat{p}
+
C_{l\hat{r}}\hat{r}
+
C_{l\delta_a}\delta_a
+
C_{l\delta_r}\delta_r
$$

$$
C_m
=
C_{m0}
+
C_{m\alpha}\alpha
+
C_{m\hat{q}}\hat{q}
+
C_{m\delta_e}\delta_e
+
C_{m\delta_{tv}}\delta_{tv}
$$

$$
C_n
=
C_{n\beta}\beta
+
C_{n\hat{p}}\hat{p}
+
C_{n\hat{r}}\hat{r}
+
C_{n\delta_a}\delta_a
+
C_{n\delta_r}\delta_r
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
X_A
=
\bar{q}S C_X
$$

$$
Y_A
=
\bar{q}S C_Y
$$

$$
Z_A
=
\bar{q}S C_Z
$$

The aerodynamic body-axis moments are:

$$
L_A
=
\bar{q}Sb C_l
$$

$$
M_A
=
\bar{q}S\bar{c} C_m
$$

$$
N_A
=
\bar{q}Sb C_n
$$

where:

- $S$ is wing reference area,
- $b$ is wingspan,
- $\bar{c}$ is mean aerodynamic chord.

---

## 10. Propulsion and Thrust Vectoring

The total engine thrust is modeled as:

$$
T
=
\delta_T T_{\max}
$$

where:

- $\delta_T$ is normalized throttle command,
- $T_{\max}$ is maximum available thrust.

For a simplified thrust-vectoring model, the thrust components may be approximated as:

$$
X_T
=
T\cos(\delta_{tv})
$$

$$
Z_T
=
-T\sin(\delta_{tv})
$$

The thrust-vectoring pitching moment may be approximated as:

$$
M_T
=
Z_T l_T
$$

where $l_T$ is the moment arm from the center of gravity to the thrust-vectoring force application point.

The total body-axis forces become:

$$
X
=
X_A + X_T
$$

$$
Y
=
Y_A
$$

$$
Z
=
Z_A + Z_T
$$

The total body-axis moments become:

$$
L
=
L_A
$$

$$
M
=
M_A + M_T
$$

$$
N
=
N_A
$$

A more advanced version may include asymmetric thrust-vectoring effects, engine gyroscopic moments, inlet effects, and nozzle dynamics.

---

## 11. Translational Equations of Motion

The nonlinear body-axis translational equations are:

$$
\dot{u}
=
rv - qw
+
\frac{X}{m}
-
g\sin(\theta)
$$

$$
\dot{v}
=
pw - ru
+
\frac{Y}{m}
+
g\sin(\phi)\cos(\theta)
$$

$$
\dot{w}
=
qu - pv
+
\frac{Z}{m}
+
g\cos(\phi)\cos(\theta)
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

For a symmetric aircraft with nonzero product of inertia $I_{xz}$, the rotational equations are:

$$
I_x\dot{p} - I_{xz}\dot{r}
=
L + I_{xz}pq - (I_z - I_y)qr
$$

$$
I_y\dot{q}
=
M + (I_z - I_x)pr - I_{xz}(p^2-r^2)
$$

$$
I_z\dot{r} - I_{xz}\dot{p}
=
N + (I_x - I_y)pq - I_{xz}qr
$$

These equations can be solved for $\dot{p}$ and $\dot{r}$ as a coupled two-equation system.

Define:

$$
A_p
=
L
+
I_{xz}pq
-
(I_z - I_y)qr
$$

$$
A_r
=
N
+
(I_x - I_y)pq
-
I_{xz}qr
$$

Then the coupled roll-yaw dynamics can be written as:

$$
\begin{bmatrix}
I_x & -I_{xz} \\
-I_{xz} & I_z
\end{bmatrix}
\begin{bmatrix}
\dot{p} \\
\dot{r}
\end{bmatrix}
=
\begin{bmatrix}
A_p \\
A_r
\end{bmatrix}
$$

Therefore:

$$
\begin{bmatrix}
\dot{p} \\
\dot{r}
\end{bmatrix}
=
\begin{bmatrix}
I_x & -I_{xz} \\
-I_{xz} & I_z
\end{bmatrix}^{-1}
\begin{bmatrix}
A_p \\
A_r
\end{bmatrix}
$$

The pitch-rate equation is:

$$
\dot{q}
=
\frac{1}{I_y}
\left[
M
+
(I_z - I_x)pr
-
I_{xz}(p^2-r^2)
\right]
$$

If the product of inertia is neglected, $I_{xz}=0$, the rotational equations simplify to:

$$
\dot{p}
=
\frac{L - (I_z - I_y)qr}{I_x}
$$

$$
\dot{q}
=
\frac{M + (I_z - I_x)pr}{I_y}
$$

$$
\dot{r}
=
\frac{N + (I_x - I_y)pq}{I_z}
$$

---

## 13. Euler Angle Kinematics

The Euler angle kinematic equations are:

$$
\dot{\phi}
=
p
+
q\sin(\phi)\tan(\theta)
+
r\cos(\phi)\tan(\theta)
$$

$$
\dot{\theta}
=
q\cos(\phi)
-
r\sin(\phi)
$$

$$
\dot{\psi}
=
\frac{q\sin(\phi)+r\cos(\phi)}{\cos(\theta)}
$$

These equations map body-axis angular rates into Euler angle rates.

A limitation of Euler angles is the singularity at:

$$
\theta
=
\pm 90^\circ
$$

For the intended flight regimes in this project, Euler angles are acceptable. For aggressive post-stall maneuvering or high-angle-of-attack simulation, quaternions would be preferable.

---

## 14. Earth-Frame Position Kinematics

The Earth-frame velocity is obtained by rotating the body-axis velocity vector into the Earth frame:

$$
\begin{bmatrix}
\dot{X}_E \\
\dot{Y}_E \\
\dot{Z}_E
\end{bmatrix}
=
C_b^e
\begin{bmatrix}
u \\
v \\
w
\end{bmatrix}
$$

where $C_b^e$ is the body-to-Earth direction cosine matrix.

Using a 3-2-1 Euler angle sequence, the direction cosine matrix is:

$$
C_b^e
=
\begin{bmatrix}
\cos(\theta)\cos(\psi) &
\sin(\phi)\sin(\theta)\cos(\psi)-\cos(\phi)\sin(\psi) &
\cos(\phi)\sin(\theta)\cos(\psi)+\sin(\phi)\sin(\psi) \\
\cos(\theta)\sin(\psi) &
\sin(\phi)\sin(\theta)\sin(\psi)+\cos(\phi)\cos(\psi) &
\cos(\phi)\sin(\theta)\sin(\psi)-\sin(\phi)\cos(\psi) \\
-\sin(\theta) &
\sin(\phi)\cos(\theta) &
\cos(\phi)\cos(\theta)
\end{bmatrix}
$$

Therefore, the Earth-frame velocity components are:

$$
\dot{X}_E
=
u\cos(\theta)\cos(\psi)
+
v\left[\sin(\phi)\sin(\theta)\cos(\psi)-\cos(\phi)\sin(\psi)\right]
+
w\left[\cos(\phi)\sin(\theta)\cos(\psi)+\sin(\phi)\sin(\psi)\right]
$$

$$
\dot{Y}_E
=
u\cos(\theta)\sin(\psi)
+
v\left[\sin(\phi)\sin(\theta)\sin(\psi)+\cos(\phi)\cos(\psi)\right]
+
w\left[\cos(\phi)\sin(\theta)\sin(\psi)-\sin(\phi)\cos(\psi)\right]
$$

$$
\dot{Z}_E
=
-u\sin(\theta)
+
v\sin(\phi)\cos(\theta)
+
w\cos(\phi)\cos(\theta)
$$

The altitude rate is approximately:

$$
\dot{h}
=
-\dot{Z}_E
$$

---

## 15. Compact State Derivative Form

The complete nonlinear state derivative vector is:

$$
\dot{x}
=
\begin{bmatrix}
\dot{u} \\
\dot{v} \\
\dot{w} \\
\dot{p} \\
\dot{q} \\
\dot{r} \\
\dot{\phi} \\
\dot{\theta} \\
\dot{\psi} \\
\dot{X}_E \\
\dot{Y}_E \\
\dot{Z}_E
\end{bmatrix}
=
f(x,u_c)
$$

where $f(x,u_c)$ is the nonlinear aircraft dynamics function implemented in MATLAB.

---

## 16. Trim Condition

A trim condition is an equilibrium point where selected accelerations are zero or nearly zero.

For steady, wings-level flight, the desired trim conditions are approximately:

$$
\dot{u}=0
$$

$$
\dot{v}=0
$$

$$
\dot{w}=0
$$

$$
\dot{p}=0
$$

$$
\dot{q}=0
$$

$$
\dot{r}=0
$$

with:

$$
\phi = 0
$$

$$
\beta = 0
$$

The trim variables may include:

$$
\alpha_{trim}, \quad \theta_{trim}, \quad \delta_{e,trim}, \quad \delta_{T,trim}, \quad \delta_{tv,trim}
$$

The trim problem can be expressed as an optimization problem:

$$
\min_z \left\| f_{trim}(z) \right\|^2
$$

where $z$ is the vector of trim unknowns.

---

## 17. Numerical Linearization

After finding a trim condition, the nonlinear dynamics can be linearized about the trim point.

Let:

$$
x = x_0 + \Delta x
$$

$$
u_c = u_{c0} + \Delta u_c
$$

The first-order linearized model is:

$$
\Delta \dot{x}
=
A\Delta x
+
B\Delta u_c
$$

where:

$$
A
=
\left.\frac{\partial f}{\partial x}\right|_{x_0,u_{c0}}
$$

$$
B
=
\left.\frac{\partial f}{\partial u_c}\right|_{x_0,u_{c0}}
$$

Using central finite differences:

$$
A_{ij}
\approx
\frac{f_i(x_0 + \epsilon e_j,u_{c0}) - f_i(x_0 - \epsilon e_j,u_{c0})}{2\epsilon}
$$

$$
B_{ij}
\approx
\frac{f_i(x_0,u_{c0} + \epsilon e_j) - f_i(x_0,u_{c0} - \epsilon e_j)}{2\epsilon}
$$

For initial MIMO controller design, the position states may be excluded so that the reduced control state is:

$$
x_c
=
\begin{bmatrix}
u \\
v \\
w \\
p \\
q \\
r \\
\phi \\
\theta \\
\psi
\end{bmatrix}
$$

---

## 18. MIMO State-Space Control Form

The reduced linear MIMO model is:

$$
\Delta \dot{x}_c
=
A_c\Delta x_c
+
B_c\Delta u_c
$$

where:

$$
\Delta u_c
=
\begin{bmatrix}
\Delta\delta_e \\
\Delta\delta_a \\
\Delta\delta_r \\
\Delta\delta_T \\
\Delta\delta_{tv}
\end{bmatrix}
$$

A full-state feedback controller has the form:

$$
\Delta u_c
=
-K\Delta x_c
$$

Therefore, the closed-loop system is:

$$
\Delta \dot{x}_c
=
(A_c - B_cK)\Delta x_c
$$

---

## 19. LQR Design

The LQR controller minimizes the quadratic cost function:

$$
J
=
\int_0^\infty
\left(
\Delta x_c^T Q \Delta x_c
+
\Delta u_c^T R \Delta u_c
\right)
dt
$$

where:

- $Q$ penalizes state deviations,
- $R$ penalizes control effort.

The feedback gain is:

$$
K
=
R^{-1}B_c^TP
$$

where $P$ is the solution of the continuous-time algebraic Riccati equation:

$$
A_c^TP
+
PA_c
-
PB_cR^{-1}B_c^TP
+
Q
=
0
$$

Using Bryson's Rule, the diagonal elements of $Q$ and $R$ can be selected as:

$$
Q_{ii}
=
\frac{1}{x_{i,max}^2}
$$

$$
R_{jj}
=
\frac{1}{u_{j,max}^2}
$$

---

## 20. Simulation Implementation Notes

The nonlinear simulation should perform the following steps at each integration time step:

1. Read the current state vector $x$.
2. Compute $V$, $\alpha$, $\beta$, $\hat{p}$, $\hat{q}$, and $\hat{r}$.
3. Compute aerodynamic coefficients.
4. Convert coefficients into aerodynamic forces and moments.
5. Add propulsion and thrust-vectoring contributions.
6. Compute translational accelerations.
7. Compute rotational accelerations.
8. Compute Euler angle rates.
9. Compute Earth-frame position rates.
10. Return $\dot{x}$ to the numerical integrator.

A typical MATLAB function interface is:

```matlab
function xdot = nonlinear_6dof_fighter(t, x, u_ctrl, params)
```

For closed-loop simulation, the controller computes $u_{ctrl}$ from the state error:

```matlab
x_error = x_control - x_trim_control;
u_cmd  = u_trim - K*x_error;
```

Actuator saturation should be applied before passing commands into the nonlinear plant.

---

## 21. Validation Checks

The model should be validated using basic engineering checks:

| Check | Expected result |
|---|---|
| Zero sideslip with symmetric controls | $Y \approx 0$, $L \approx 0$, $N \approx 0$ |
| Positive elevator/stabilator command | Produces expected pitching moment sign based on chosen convention |
| Positive aileron command | Produces expected roll moment sign based on chosen convention |
| Positive rudder command | Produces expected yaw moment sign based on chosen convention |
| Trim simulation | States remain near equilibrium |
| Open-loop perturbation | Aircraft shows natural dynamic response |
| Closed-loop LQR simulation | Perturbations decay with bounded control inputs |

---

## 22. Known Limitations

This first nonlinear 6-DOF implementation has several limitations:

- aerodynamic coefficients are simplified and not based on real F-22 data,
- no lookup tables are used for Mach number, altitude, angle of attack, or control-surface scheduling,
- actuator dynamics are initially neglected,
- engine spool dynamics are initially neglected,
- structural flexibility is neglected,
- sensor noise and estimation dynamics are not included,
- the Earth is modeled as locally flat,
- Euler angles are used instead of quaternions,
- aggressive post-stall aerodynamics are not represented.

These limitations should be documented clearly in the GitHub repository.

---

## 23. Summary

The nonlinear 6-DOF model provides the foundation for the F-22-inspired MIMO fighter aircraft simulation. It combines body-axis translational dynamics, rotational rigid-body dynamics, Euler attitude kinematics, Earth-frame position kinematics, aerodynamic force/moment modeling, propulsion modeling, and thrust-vectoring effects.

This model supports:

- open-loop fighter aircraft simulation,
- trim and linearization,
- MIMO LQR design,
- closed-loop attitude stabilization,
- control-surface response analysis,
- future automatic landing and guidance development.

The project is intended to demonstrate professional aerospace modeling and control-design workflow using public-data-inspired assumptions and transparent educational documentation.
