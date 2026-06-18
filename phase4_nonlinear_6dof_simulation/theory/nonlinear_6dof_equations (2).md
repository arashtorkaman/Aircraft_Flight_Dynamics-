# Nonlinear 6-DOF Aircraft Equations of Motion

## Project Context

This document defines the nonlinear six-degree-of-freedom equations of motion used for the **F-22-inspired MIMO fighter aircraft simulation**. The equations describe the rigid-body translational, rotational, attitude, and position dynamics of a fixed-wing aircraft using body-axis velocities, angular rates, Euler angles, and Earth-frame position states.

The model is intended for educational flight-dynamics, control-design, and simulation purposes. It is **not** a representation of the actual F-22 flight-control system, proprietary aerodynamic database, or classified aircraft model.

---

## 1. Purpose of the 6-DOF Model

A six-degree-of-freedom aircraft model captures the coupled nonlinear motion of the aircraft in three translational and three rotational axes.

The six degrees of freedom are:

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

---

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

where:

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

where:

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

The sideslip angle is:

$$
\beta = \sin^{-1}\left(\frac{v}{V}\right)
$$

For numerical robustness, the implementation should prevent division by zero when $V$ is very small.

A practical implementation is:

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
C_X = C_{X0} + C_{X_{\alpha}}\alpha + C_{X_{\delta_e}}\delta_e
$$

$$
C_Y = C_{Y_{\beta}}\beta + C_{Y_{\hat{p}}}\hat{p} + C_{Y_{\hat{r}}}\hat{r} + C_{Y_{\delta_a}}\delta_a + C_{Y_{\delta_r}}\delta_r
$$

$$
C_Z = C_{Z0} + C_{Z_{\alpha}}\alpha + C_{Z_{\hat{q}}}\hat{q} + C_{Z_{\delta_e}}\delta_e
$$

$$
C_l = C_{l_{\beta}}\beta + C_{l_{\hat{p}}}\hat{p} + C_{l_{\hat{r}}}\hat{r} + C_{l_{\delta_a}}\delta_a + C_{l_{\delta_r}}\delta_r
$$

$$
C_m = C_{m0} + C_{m_{\alpha}}\alpha + C_{m_{\hat{q}}}\hat{q} + C_{m_{\delta_e}}\delta_e + C_{m_{\delta_{tv}}}\delta_{tv}
$$

$$
C_n = C_{n_{\beta}}\beta + C_{n_{\hat{p}}}\hat{p} + C_{n_{\hat{r}}}\hat{r} + C_{n_{\delta_a}}\delta_a + C_{n_{\delta_r}}\delta_r
$$

The signs of the main stability derivatives should be chosen to produce physically reasonable aircraft behavior:

| Stability derivative | Required sign | Physical meaning |
|---|---:|---|
| $C_{m_{\alpha}}$ | $< 0$ | Longitudinal static stability in the simplified model |
| $C_{m_{\hat{q}}}$ | $< 0$ | Pitch-rate damping |
| $C_{l_{\hat{p}}}$ | $< 0$ | Roll-rate damping |
| $C_{n_{\hat{r}}}$ | $< 0$ | Yaw-rate damping |
| $C_{n_{\beta}}$ | $> 0$ | Directional weathercock stability |

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
A_p = L + I_{xz}pq - (I_z - I_y)qr
$$

$$
A_r = N + (I_x - I_y)pq - I_{xz}qr
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

and:

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
\cos\theta\cos\psi &
\sin\phi\sin\theta\cos\psi - \cos\phi\sin\psi &
\cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi \\
\cos\theta\sin\psi &
\sin\phi\sin\theta\sin\psi + \cos\phi\cos\psi &
\cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi \\
-\sin\theta &
\sin\phi\cos\theta &
\cos\phi\cos\theta
\end{bmatrix}
$$

Therefore, the Earth-frame velocity components are:

$$
\begin{aligned}
\dot{X}_E &=
u\cos\theta\cos\psi
+ v(\sin\phi\sin\theta\cos\psi - \cos\phi\sin\psi)
+ w(\cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi)
\\[6pt]
\dot{Y}_E &=
u\cos\theta\sin\psi
+ v(\sin\phi\sin\theta\sin\psi + \cos\phi\cos\psi)
+ w(\cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi)
\\[6pt]
\dot{Z}_E &=
-u\sin\theta
+ v\sin\phi\cos\theta
+ w\cos\phi\cos\theta
\end{aligned}
$$

For a North-East-Down convention, $Z_E$ is positive downward. Altitude is therefore computed as:

$$
h = -Z_E
$$

and the altitude rate is:

$$
\dot{h} = -\dot{Z}_E
$$

---

## 15. Complete State Derivative Vector

The complete state derivative is:

$$
\dot{x} =
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
$$

This vector is computed inside the nonlinear plant function:

```matlab
xdot = nonlinear_6dof_fighter(t, x, u_ctrl, params);
```

---

## 16. MATLAB Implementation Structure

The nonlinear plant should follow this structure:

```matlab
function xdot = nonlinear_6dof_fighter(t, x, u_ctrl, params)

% Unpack states
u     = x(1);
v     = x(2);
w     = x(3);
p     = x(4);
q     = x(5);
r     = x(6);
phi   = x(7);
theta = x(8);
psi   = x(9);
XE    = x(10);
YE    = x(11);
ZE    = x(12);

% Unpack controls
de  = u_ctrl(1);
da  = u_ctrl(2);
dr  = u_ctrl(3);
dT  = u_ctrl(4);
dtv = u_ctrl(5);

% Compute air data
V = sqrt(u^2 + v^2 + w^2);
V = max(V, 1e-3);
alpha = atan2(w, u);
beta = asin(max(min(v/V, 1), -1));

% Compute dynamic pressure
qbar = 0.5 * params.rho * V^2;

% Compute aerodynamic coefficients
coeff = aero_coefficients_fighter(alpha, beta, p, q, r, de, da, dr, dtv, params);

% Convert coefficients to forces and moments
XA = qbar * params.S * coeff.CX;
YA = qbar * params.S * coeff.CY;
ZA = qbar * params.S * coeff.CZ;

LA = qbar * params.S * params.b    * coeff.Cl;
MA = qbar * params.S * params.cbar * coeff.Cm;
NA = qbar * params.S * params.b    * coeff.Cn;

% Propulsion model
T  = dT * params.Tmax;
XT = T * cos(dtv);
ZT = -T * sin(dtv);
MT = ZT * params.lt;

% Total forces and moments
X = XA + XT;
Y = YA;
Z = ZA + ZT;

L = LA;
M = MA + MT;
N = NA;

% Translational dynamics
udot = r*v - q*w + X/params.m - params.g*sin(theta);
vdot = p*w - r*u + Y/params.m + params.g*sin(phi)*cos(theta);
wdot = q*u - p*v + Z/params.m + params.g*cos(phi)*cos(theta);

% Rotational dynamics with Ixz coupling
Ap = L + params.Ixz*p*q - (params.Iz - params.Iy)*q*r;
Ar = N + (params.Ix - params.Iy)*p*q - params.Ixz*q*r;

Ipr = [params.Ix, -params.Ixz; -params.Ixz, params.Iz];
p_r_dot = Ipr \ [Ap; Ar];

pdot = p_r_dot(1);
rdot = p_r_dot(2);

qdot = (M + (params.Iz - params.Ix)*p*r - params.Ixz*(p^2 - r^2)) / params.Iy;

% Euler angle kinematics
phidot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
thetadot = q*cos(phi) - r*sin(phi);
psidot   = (q*sin(phi) + r*cos(phi)) / cos(theta);

% Position kinematics
Cbe = [
    cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
    -sin(theta),          sin(phi)*cos(theta),                                  cos(phi)*cos(theta)
];

posdot = Cbe * [u; v; w];

XEdot = posdot(1);
YEdot = posdot(2);
ZEdot = posdot(3);

% Full state derivative
xdot = [
    udot;
    vdot;
    wdot;
    pdot;
    qdot;
    rdot;
    phidot;
    thetadot;
    psidot;
    XEdot;
    YEdot;
    ZEdot
];

end
```

---

## 17. Simulation Integration

The nonlinear equations can be integrated using `ode45`:

```matlab
tspan = [0 20];

[t, x] = ode45(@(t,x) nonlinear_6dof_fighter(t, x, u_ctrl0, params), tspan, x0);
```

For closed-loop control, the control command is updated inside the integration function:

```matlab
[t, x] = ode45(@(t,x) closed_loop_6dof(t, x, x_trim, u_trim, K, params), tspan, x0);
```

where:

```matlab
u_ctrl = u_trim - K*(x_control - x_trim_control);
```

---

## 18. Connection to MIMO Linearization

The nonlinear model has the general form:

$$
\dot{x} = f(x,u_c)
$$

Around a trim condition $(x_0,u_0)$, the model can be linearized as:

$$
\Delta\dot{x} = A\Delta x + B\Delta u_c
$$

where:

$$
A = \left.\frac{\partial f}{\partial x}\right|_{x_0,u_0}
$$

$$
B = \left.\frac{\partial f}{\partial u_c}\right|_{x_0,u_0}
$$

For numerical linearization:

$$
A_{ij} \approx \frac{f_i(x_0 + \epsilon e_j, u_0) - f_i(x_0 - \epsilon e_j, u_0)}{2\epsilon}
$$

$$
B_{ij} \approx \frac{f_i(x_0, u_0 + \epsilon e_j) - f_i(x_0, u_0 - \epsilon e_j)}{2\epsilon}
$$

The resulting $A$ and $B$ matrices are used for MIMO control design.

---

## 19. Connection to MIMO LQR Control

After linearization, the MIMO LQR controller is designed as:

$$
\Delta u_c = -K\Delta x
$$

where:

$$
K = \text{lqr}(A,B,Q,R)
$$

The closed-loop linear system is:

$$
\Delta\dot{x} = (A - BK)\Delta x
$$

The same controller can then be tested on the nonlinear aircraft plant:

$$
u_c = u_{c0} - K(x - x_0)
$$

This validates whether the linear controller performs acceptably when applied to the nonlinear 6-DOF dynamics.

---

## 20. Recommended Validation Plots

The nonlinear 6-DOF model should be validated using the following plots:

1. Body-axis velocities:

   $$
   u(t), \quad v(t), \quad w(t)
   $$

2. Angular rates:

   $$
   p(t), \quad q(t), \quad r(t)
   $$

3. Euler angles:

   $$
   \phi(t), \quad \theta(t), \quad \psi(t)
   $$

4. Earth-frame position:

   $$
   X_E(t), \quad Y_E(t), \quad Z_E(t)
   $$

5. Altitude:

   $$
   h(t) = -Z_E(t)
   $$

6. Control inputs:

   $$
   \delta_e(t), \quad \delta_a(t), \quad \delta_r(t), \quad \delta_T(t), \quad \delta_{tv}(t)
   $$

7. Three-dimensional trajectory:

   $$
   X_E(t), \quad Y_E(t), \quad h(t)
   $$

---

## 21. Known Limitations

This 6-DOF model is intentionally simplified. The following effects are not included in the first version:

- compressibility and Mach-dependent aerodynamic coefficients,
- high-angle-of-attack vortex effects,
- post-stall aerodynamics,
- actuator bandwidth and rate limits,
- sensor noise and estimation filters,
- flexible-body structural dynamics,
- fuel burn and mass variation,
- detailed propulsion dynamics,
- proprietary F-22 aerodynamic data,
- real F-22 flight-control laws,
- real thrust-vectoring control allocation.

These limitations should be clearly stated in the project documentation.

---

## 22. Summary

The nonlinear 6-DOF model provides the simulation foundation for the F-22-inspired MIMO fighter control project. It combines:

- nonlinear body-axis translational dynamics,
- nonlinear rotational dynamics,
- Euler angle attitude kinematics,
- Earth-frame position kinematics,
- aerodynamic force and moment modeling,
- propulsion and simplified thrust-vectoring effects,
- numerical linearization for MIMO controller design.

This model is the bridge between simplified linear state-space control and realistic aircraft simulation. It allows the same aircraft plant to support open-loop analysis, closed-loop LQR control, MIMO response studies, trajectory plotting, and future automatic landing/guidance development.
