# Nonlinear 6-DOF Aircraft Equations of Motion

## Project Context

This document defines the nonlinear six-degree-of-freedom equations of motion used for the **F-22-inspired MIMO fighter aircraft simulation**.

The equations describe the rigid-body translational, rotational, attitude, and position dynamics of a fixed-wing aircraft using body-axis velocities, angular rates, Euler angles, and Earth-frame position states.

> **Important limitation:**  
> This model is intended for educational flight-dynamics, control-design, and simulation purposes. It is **not** a representation of the actual F-22 flight-control system, proprietary aerodynamic database, or classified aircraft model.

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
\mathbf{V}_b =
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
\boldsymbol{\omega}_b =
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

The Earth frame is assumed to be locally flat and inertial for the purposes of this simulation.

The Earth-frame position vector is:

$$
\mathbf{r}_e =
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

In this convention, altitude is approximately:

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
u_c =
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

In implementation, the two-argument arctangent should be used:

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

In the first simulation version, constant air density may be used:

$$
\rho = 1.225 \ \mathrm{kg/m^3}
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
\hat{p} = \frac{p b}{2V}
$$

Pitch-rate term:

$$
\hat{q} = \frac{q \bar{c}}{2V}
$$

Yaw-rate term:

$$
\hat{r} = \frac{r b}{2V}
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
C_X,\ C_Y,\ C_Z
$$

The body-axis moment coefficients are:

$$
C_l,\ C_m,\ C_n
$$

A simplified coefficient model may be written as:

$$
C_X =
C_{X_0}
+ C_{X_\alpha}\alpha
+ C_{X_{\delta_e}}\delta_e
$$

$$
C_Y =
C_{Y_\beta}\beta
+ C_{Y_{\hat{p}}}\hat{p}
+ C_{Y_{\hat{r}}}\hat{r}
+ C_{Y_{\delta_a}}\delta_a
+ C_{Y_{\delta_r}}\delta_r
$$

$$
C_Z =
C_{Z_0}
+ C_{Z_\alpha}\alpha
+ C_{Z_{\hat{q}}}\hat{q}
+ C_{Z_{\delta_e}}\delta_e
$$

$$
C_l =
C_{l_\beta}\beta
+ C_{l_{\hat{p}}}\hat{p}
+ C_{l_{\hat{r}}}\hat{r}
+ C_{l_{\delta_a}}\delta_a
+ C_{l_{\delta_r}}\delta_r
$$

$$
C_m =
C_{m_0}
+ C_{m_\alpha}\alpha
+ C_{m_{\hat{q}}}\hat{q}
+ C_{m_{\delta_e}}\delta_e
+ C_{m_{\delta_{tv}}}\delta_{tv}
$$

$$
C_n =
C_{n_\beta}\beta
+ C_{n_{\hat{p}}}\hat{p}
+ C_{n_{\hat{r}}}\hat{r}
+ C_{n_{\delta_a}}\delta_a
+ C_{n_{\delta_r}}\delta_r
$$

The signs of the main stability derivatives should be chosen to produce physically reasonable aircraft behavior.

| Stability derivative | Required sign | Physical meaning |
|---|---:|---|
| $C_{m_\alpha}$ | $< 0$ | Longitudinal static stability in the simplified model |
| $C_{m_{\hat{q}}}$ | $< 0$ | Pitch-rate damping |
| $C_{l_{\hat{p}}}$ | $< 0$ | Roll-rate damping |
| $C_{n_{\hat{r}}}$ | $< 0$ | Yaw-rate damping |
| $C_{n_\beta}$ | $> 0$ | Directional weathercock stability |

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
T = \delta_T T_{\max}
$$

where:

- $\delta_T$ is normalized throttle command,
- $T_{\max}$ is maximum available thrust.

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
\dot{u}
=
rv - qw + \frac{X}{m} - g\sin\theta
$$

$$
\dot{v}
=
pw - ru + \frac{Y}{m} + g\sin\phi\cos\theta
$$

$$
\dot{w}
=
qu - pv + \frac{Z}{m} + g\cos\phi\cos\theta
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
A_p =
L + I_{xz}pq - (I_z - I_y)qr
$$

$$
A_r =
N + (I_x - I_y)pq - I_{xz}qr
$$

Then:

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

and therefore:

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
+ (I_z - I_x)pr
- I_{xz}(p^2-r^2)
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
+ q\sin\phi\tan\theta
+ r\cos\phi\tan\theta
$$

$$
\dot{\theta}
=
q\cos\phi
- r\sin\phi
$$

$$
\dot{\psi}
=
\frac{
q\sin\phi
+ r\cos\phi
}{\cos\theta}
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

where $C_b^e$ is the body-to-Earth direction cosine matrix.

Using a 3-2-1 Euler angle sequence, the direction cosine matrix is:

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
\begin{aligned}
\dot{X}_E
&=
u\cos\theta\cos\psi
+ v(\sin\phi\sin\theta\cos\psi - \cos\phi\sin\psi)
+ w(\cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi)
\\
\dot{Y}_E
&=
u\cos\theta\sin\psi
+ v(\sin\phi\sin\theta\sin\psi + \cos\phi\cos\psi)
+ w(\cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi)
\\
\dot{Z}_E
&=
-u\sin\theta
+ v\sin\phi\cos\theta
+ w\cos\phi\cos\theta
\end{aligned}
$$

---

## 15. Complete Nonlinear State Derivative

The complete nonlinear state derivative is:

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

where:

- $x$ is the nonlinear aircraft state vector,
- $u_c$ is the control input vector,
- $f(x,u_c)$ is the nonlinear 6-DOF aircraft model.

---

## 16. MATLAB Implementation Notes

The nonlinear dynamics function should follow the structure:

```matlab
function xdot = nonlinear_6dof_fighter(t, x, u_ctrl, params)

% Extract states
u     = x(1);
v     = x(2);
w     = x(3);
p     = x(4);
q     = x(5);
r     = x(6);
phi   = x(7);
theta = x(8);
psi   = x(9);

% Extract controls
de  = u_ctrl(1);
da  = u_ctrl(2);
dr  = u_ctrl(3);
dT  = u_ctrl(4);
dtv = u_ctrl(5);

% Compute air-data quantities
V = sqrt(u^2 + v^2 + w^2);
V = max(V, 1e-3);

alpha = atan2(w,u);
beta  = asin(max(min(v/V,1),-1));

qbar = 0.5 * params.rho * V^2;

phat = p * params.b    / (2*V);
qhat = q * params.cbar / (2*V);
rhat = r * params.b    / (2*V);

% Compute aerodynamic coefficients
% Compute forces and moments
% Compute translational dynamics
% Compute rotational dynamics
% Compute Euler angle rates
% Compute Earth-frame position rates

xdot = zeros(12,1);

end
```

---

## 17. Validation Checks

The model should be validated using the following checks:

1. **Dimensional consistency**  
   Forces should be in newtons, moments in newton-meters, velocities in m/s, rates in rad/s, and accelerations in m/s² or rad/s².

2. **Zero-control sanity check**  
   With symmetric initial conditions and zero lateral inputs, the lateral states should remain close to zero.

3. **Trim consistency check**  
   At a trimmed condition, $\dot{x}$ should be close to zero for the velocity, attitude-rate, and angular-rate states.

4. **Small-perturbation check**  
   Near trim, the nonlinear response should approximately match the linearized model.

5. **Closed-loop check**  
   With MIMO LQR feedback, the aircraft should return toward the commanded trim or attitude state after small perturbations.

---

## 18. Known Limitations

This first model intentionally simplifies several effects:

- no proprietary F-22 aerodynamic database,
- no real flight-control law or gain schedule,
- no actuator bandwidth or rate limits in the first version,
- no flexible-body dynamics,
- no fuel burn or center-of-gravity shift,
- no nonlinear aerodynamic lookup tables,
- no compressibility correction,
- no post-stall vortex-lift modeling,
- no atmospheric turbulence or wind gusts in the first version.

These limitations should be clearly stated in the project README and documentation.

---

## 19. Future Improvements

Future versions may add:

- trim solver using `fmincon` or `lsqnonlin`,
- numerical linearization about multiple flight conditions,
- gain-scheduled LQR control,
- actuator dynamics and saturation,
- thrust-vectoring control allocation,
- nonlinear aerodynamic lookup tables,
- wind and turbulence models,
- automatic landing guidance laws,
- 3D trajectory visualization,
- Simulink Aerospace Blockset implementation.

---

## Summary

This document defines a complete nonlinear 6-DOF aircraft model suitable for an F-22-inspired MIMO fighter aircraft simulation. The model includes body-axis translational dynamics, rotational rigid-body dynamics, Euler angle kinematics, Earth-frame position kinematics, aerodynamic force and moment coefficients, thrust-vectoring approximations, and validation requirements.

The model is not a real F-22 representation. It is a structured educational simulation intended to demonstrate flight dynamics, nonlinear modeling, numerical linearization, and multivariable control design.
