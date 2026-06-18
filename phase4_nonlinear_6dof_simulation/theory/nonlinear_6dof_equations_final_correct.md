# Nonlinear 6-DOF Aircraft Equations of Motion

## Project Context

This document defines the nonlinear six-degree-of-freedom equations of motion used for the F-22-inspired MIMO fighter aircraft simulation.

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
V_b =
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
\omega_b =
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
r_e =
\begin{bmatrix}
X_E \\
Y_E \\
Z_E
\end{bmatrix}
$$

where:

- $X_E$ is forward/inertial position,
- $Y_E$ is lateral/inertial position,
- $Z_E$ is vertical/down position.

Using the positive-down Earth-frame convention, altitude is approximately:

$$
h = -Z_E
$$

---

## 3. Full Nonlinear State Vector

The nonlinear simulation state vector is:

$$
x =
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

The states are:

- $u$ = forward body-axis velocity, in m/s
- $v$ = lateral body-axis velocity, in m/s
- $w$ = vertical body-axis velocity, in m/s
- $p$ = roll rate, in rad/s
- $q$ = pitch rate, in rad/s
- $r$ = yaw rate, in rad/s
- $\phi$ = roll angle, in rad
- $\theta$ = pitch angle, in rad
- $\psi$ = yaw or heading angle, in rad
- $X_E$ = Earth-frame forward position, in m
- $Y_E$ = Earth-frame lateral position, in m
- $Z_E$ = Earth-frame vertical/down position, in m

---

## 4. Control Input Vector

For the F-22-inspired MIMO fighter model, the control input vector is:

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

- $\delta_e$ = elevator or stabilator command, in rad
- $\delta_a$ = aileron command, in rad
- $\delta_r$ = rudder command, in rad
- $\delta_T$ = normalized throttle command
- $\delta_{tv}$ = pitch thrust-vectoring command, in rad

The first implementation treats these commands as idealized actuator inputs. Later versions may add actuator dynamics, rate limits, dead zones, saturation, and control-allocation logic.

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

In implementation, use the two-argument arctangent:

$$
\alpha = \operatorname{atan2}(w,u)
$$

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
\rho = 1.225 \; \mathrm{kg/m^3}
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

A simplified coefficient model may be written as follows.

### 8.1 Force Coefficients

$$
C_X = C_{X0} + C_{X\alpha}\alpha + C_{X\delta_e}\delta_e
$$

$$
C_Y = C_{Y\beta}\beta + C_{Y\hat{p}}\hat{p} + C_{Y\hat{r}}\hat{r} + C_{Y\delta_a}\delta_a + C_{Y\delta_r}\delta_r
$$

$$
C_Z = C_{Z0} + C_{Z\alpha}\alpha + C_{Z\hat{q}}\hat{q} + C_{Z\delta_e}\delta_e
$$

### 8.2 Moment Coefficients

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

- $C_{m\alpha} < 0$ for longitudinal static stability in the simplified model.
- $C_{m\hat{q}} < 0$ for pitch-rate damping.
- $C_{l\hat{p}} < 0$ for roll-rate damping.
- $C_{n\hat{r}} < 0$ for yaw-rate damping.
- $C_{n\beta} > 0$ for directional weathercock stability.

---

## 9. Aerodynamic Forces and Moments

The aerodynamic body-axis forces are computed from the force coefficients:

$$
X_A = \bar{q} S C_X
$$

$$
Y_A = \bar{q} S C_Y
$$

$$
Z_A = \bar{q} S C_Z
$$

The aerodynamic body-axis moments are:

$$
L_A = \bar{q} S b C_l
$$

$$
M_A = \bar{q} S \bar{c} C_m
$$

$$
N_A = \bar{q} S b C_n
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

For a simplified pitch thrust-vectoring model, the thrust components may be approximated as:

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

The total body-axis forces are:

$$
X = X_A + X_T
$$

$$
Y = Y_A
$$

$$
Z = Z_A + Z_T
$$

The total moments are:

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
A_p = L + I_{xz}pq - (I_z - I_y)qr
$$

$$
A_r = N + (I_x - I_y)pq - I_{xz}qr
$$

Then the coupled roll-yaw rate equations can be written as:

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
M + (I_z - I_x)pr - I_{xz}(p^2-r^2)
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
p + q\sin\phi\tan\theta + r\cos\phi\tan\theta
$$

$$
\dot{\theta}
=
q\cos\phi - r\sin\phi
$$

$$
\dot{\psi}
=
\frac{q\sin\phi + r\cos\phi}{\cos\theta}
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

Using a 3-2-1 Euler angle sequence, the body-to-Earth direction cosine matrix is:

$$
C_b^e =
\begin{bmatrix}
\cos\theta\cos\psi
&
\sin\phi\sin\theta\cos\psi - \cos\phi\sin\psi
&
\cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi
\\
\cos\theta\sin\psi
&
\sin\phi\sin\theta\sin\psi + \cos\phi\cos\psi
&
\cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi
\\
-\sin\theta
&
\sin\phi\cos\theta
&
\cos\phi\cos\theta
\end{bmatrix}
$$

Therefore, the Earth-frame velocity components are:

$$
\dot{X}_E
=
u\cos\theta\cos\psi
+ v(\sin\phi\sin\theta\cos\psi - \cos\phi\sin\psi)
+ w(\cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi)
$$

$$
\dot{Y}_E
=
u\cos\theta\sin\psi
+ v(\sin\phi\sin\theta\sin\psi + \cos\phi\cos\psi)
+ w(\cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi)
$$

$$
\dot{Z}_E
=
-u\sin\theta
+ v\sin\phi\cos\theta
+ w\cos\phi\cos\theta
$$

---

## 15. Complete Nonlinear State Derivative

The nonlinear state derivative is:

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

The function $f(x,u_c)$ represents the complete nonlinear 6-DOF aircraft model.

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
\Delta\dot{x} = A\Delta x + B\Delta u_c
$$

where:

$$
A = \left.\frac{\partial f}{\partial x}\right|_{x_0,u_{c0}}
$$

$$
B = \left.\frac{\partial f}{\partial u_c}\right|_{x_0,u_{c0}}
$$

In the implementation, $A$ and $B$ can be computed using finite-difference numerical linearization.

---

## 17. Model Limitations

This 6-DOF model is intentionally simplified. It does not include:

- proprietary F-22 aerodynamic data,
- real F-22 flight-control laws,
- full aerodynamic lookup tables,
- actuator bandwidth and rate limits,
- flexible-body dynamics,
- structural modes,
- sensor dynamics,
- turbulence or gust models,
- compressibility effects,
- high-angle-of-attack vortex-dominated aerodynamics,
- stall or post-stall dynamics.

The model is appropriate for educational simulation, portfolio demonstration, and control-design workflow development.
