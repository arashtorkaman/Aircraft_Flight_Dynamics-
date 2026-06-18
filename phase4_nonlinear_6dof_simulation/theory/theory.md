# Theory — MIMO 6-DOF Flight Dynamics and Control of an F-22-Inspired Fighter Aircraft

## 1. Project Purpose

This project develops a **public-data-inspired, F-22-style fighter aircraft simulation** for portfolio demonstration in flight dynamics, nonlinear simulation, and multivariable control.

The goal is **not** to reproduce the real F-22 flight-control system. The real F-22 aerodynamic database, mass properties, actuator dynamics, control laws, thrust-vectoring schedules, and gain schedules are not publicly available. Therefore, this project uses representative fighter-aircraft parameters and simplified aerodynamic derivatives to demonstrate the engineering workflow.

The main objectives are:

1. Develop a nonlinear 6-degree-of-freedom rigid-body aircraft model.
2. Define aerodynamic force and moment coefficients for a high-performance fighter-style aircraft.
3. Include multiple control effectors: elevator/stabilator, aileron, rudder, throttle, and optional pitch thrust vectoring.
4. Trim the nonlinear aircraft model around a selected flight condition.
5. Numerically linearize the nonlinear model about trim.
6. Design a multivariable LQR controller.
7. Simulate and compare open-loop and closed-loop aircraft response.
8. Generate professional plots for GitHub documentation.

---

## 2. Aircraft Modeling Philosophy

The aircraft is modeled as a rigid body with six degrees of freedom:

- Three translational degrees of freedom.
- Three rotational degrees of freedom.

The model includes:

- Body-axis translational velocities.
- Body-axis angular rates.
- Euler attitude angles.
- Earth-frame position.
- Aerodynamic forces and moments.
- Propulsive force.
- Optional thrust-vectoring pitching moment.

The simulation follows the structure:

```text
State vector -> aerodynamic model -> forces and moments -> 6-DOF equations -> state derivatives
```

The control loop follows:

```text
Reference command -> state error -> MIMO controller -> control effector commands -> nonlinear aircraft model
```

---

## 3. Coordinate Frames

### 3.1 Body Frame

The aircraft body frame is fixed to the aircraft:

- \(x_b\): positive forward through the nose.
- \(y_b\): positive out the right wing.
- \(z_b\): positive downward.

The body-axis velocity vector is:

$$
V_b = \begin{bmatrix} u \\ v \\ w \end{bmatrix}
$$

where:

- \(u\): forward body-axis velocity.
- \(v\): lateral body-axis velocity.
- \(w\): vertical body-axis velocity, positive downward.

The body-axis angular-rate vector is:

$$
\omega_b = \begin{bmatrix} p \\ q \\ r \end{bmatrix}
$$

where:

- \(p\): roll rate.
- \(q\): pitch rate.
- \(r\): yaw rate.

### 3.2 Earth Frame

The Earth frame is used for aircraft position:

- \(X_E\): downrange position.
- \(Y_E\): lateral position.
- \(Z_E\): vertical position, positive downward.

Altitude is related to \(Z_E\) by:

$$
h = -Z_E
$$

---

## 4. Nonlinear 6-DOF State Vector

The full nonlinear state vector is:

$$
x = \begin{bmatrix} u & v & w & p & q & r & \phi & \theta & \psi & X_E & Y_E & Z_E \end{bmatrix}^T
$$

where:

| State | Description | Unit |
|---|---|---|
| \(u\) | Forward body velocity | m/s |
| \(v\) | Lateral body velocity | m/s |
| \(w\) | Vertical body velocity | m/s |
| \(p\) | Roll rate | rad/s |
| \(q\) | Pitch rate | rad/s |
| \(r\) | Yaw rate | rad/s |
| \(\phi\) | Roll angle | rad |
| \(\theta\) | Pitch angle | rad |
| \(\psi\) | Yaw angle | rad |
| \(X_E\) | Earth-frame downrange position | m |
| \(Y_E\) | Earth-frame lateral position | m |
| \(Z_E\) | Earth-frame vertical position, positive downward | m |

---

## 5. Control Input Vector

The MIMO control input vector is:

$$
u_c = \begin{bmatrix} \delta_e & \delta_a & \delta_r & \delta_T & \delta_{tv} \end{bmatrix}^T
$$

where:

| Input | Description | Unit |
|---|---|---|
| \(\delta_e\) | Elevator or stabilator command | rad |
| \(\delta_a\) | Aileron command | rad |
| \(\delta_r\) | Rudder command | rad |
| \(\delta_T\) | Normalized throttle command | nondimensional |
| \(\delta_{tv}\) | Pitch thrust-vectoring command | rad |

For the first implementation, the aircraft is controlled using elevator/stabilator, aileron, rudder, throttle, and optional pitch-axis thrust vectoring.

---

## 6. Flight Condition Variables

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

The dynamic pressure is:

$$
\bar{q} = \frac{1}{2}\rho V^2
$$

The nondimensional angular rates are:

$$
\hat{p} = \frac{p b}{2V}
$$

$$
\hat{q} = \frac{q \bar{c}}{2V}
$$

$$
\hat{r} = \frac{r b}{2V}
$$

where:

- \(b\): wingspan.
- \(\bar{c}\): mean aerodynamic chord.
- \(\rho\): air density.

---

## 7. Translational Equations of Motion

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

- \(X\): total force along the body \(x_b\)-axis.
- \(Y\): total force along the body \(y_b\)-axis.
- \(Z\): total force along the body \(z_b\)-axis.
- \(m\): aircraft mass.
- \(g\): gravitational acceleration.

These equations include inertial coupling terms such as \(rv\), \(qw\), \(pw\), \(ru\), \(qu\), and \(pv\). These coupling terms are one reason a fighter aircraft must be treated as a multivariable dynamic system.

---

## 8. Rotational Equations of Motion

The aircraft inertia tensor is represented using:

$$
I = \begin{bmatrix} I_x & 0 & -I_{xz} \\ 0 & I_y & 0 \\ -I_{xz} & 0 & I_z \end{bmatrix}
$$

The nonlinear rotational equations are:

$$
I_x\dot{p} - I_{xz}\dot{r} = L + I_{xz}pq - (I_z - I_y)qr
$$

$$
I_y\dot{q} = M + (I_z - I_x)pr - I_{xz}(p^2-r^2)
$$

$$
I_z\dot{r} - I_{xz}\dot{p} = N + (I_x - I_y)pq - I_{xz}qr
$$

where:

- \(L\): rolling moment.
- \(M\): pitching moment.
- \(N\): yawing moment.
- \(I_x\), \(I_y\), \(I_z\): principal mass moments of inertia.
- \(I_{xz}\): product of inertia.

For numerical implementation, the first and third rotational equations can be solved simultaneously for \(\dot{p}\) and \(\dot{r}\).

---

## 9. Euler Angle Kinematics

The relationship between body angular rates and Euler angle rates is:

$$
\dot{\phi} = p + q\sin\phi\tan\theta + r\cos\phi\tan\theta
$$

$$
\dot{\theta} = q\cos\phi - r\sin\phi
$$

$$
\dot{\psi} = \frac{q\sin\phi + r\cos\phi}{\cos\theta}
$$

These equations become singular when \(\theta = \pm 90^\circ\). For this portfolio phase, Euler angles are acceptable because the initial simulations are performed near conventional flight attitudes. A future improvement would replace Euler angles with quaternions.

---

## 10. Position Kinematics

The Earth-frame position rate is obtained by transforming body-axis velocity into the Earth frame:

$$
\begin{bmatrix} \dot{X}_E \\ \dot{Y}_E \\ \dot{Z}_E \end{bmatrix} = C_b^e \begin{bmatrix} u \\ v \\ w \end{bmatrix}
$$

where \(C_b^e\) is the body-to-Earth direction cosine matrix.

For a 3-2-1 Euler-angle rotation sequence, the transformation can be written as:

$$
C_b^e = R_z(\psi)R_y(\theta)R_x(\phi)
$$

where \(R_x\), \(R_y\), and \(R_z\) are elementary rotation matrices about the roll, pitch, and yaw axes.

---

## 11. Aerodynamic Force Model

The aerodynamic forces are modeled using nondimensional coefficients:

$$
X_{aero} = \bar{q} S C_X
$$

$$
Y_{aero} = \bar{q} S C_Y
$$

$$
Z_{aero} = \bar{q} S C_Z
$$

where:

- \(S\): wing reference area.
- \(C_X\): body-axis axial force coefficient.
- \(C_Y\): body-axis side-force coefficient.
- \(C_Z\): body-axis normal-force coefficient.

A simplified lift and drag model is:

$$
C_L = C_{L0} + C_{L\alpha}\alpha + C_{L\hat{q}}\hat{q} + C_{L\delta_e}\delta_e
$$

$$
C_D = C_{D0} + K C_L^2
$$

The lift and drag coefficients are converted into body-axis coefficients using:

$$
C_X = -C_D\cos\alpha + C_L\sin\alpha
$$

$$
C_Z = -C_D\sin\alpha - C_L\cos\alpha
$$

The lateral force coefficient is modeled as:

$$
C_Y = C_{Y\beta}\beta + C_{Y\hat{p}}\hat{p} + C_{Y\hat{r}}\hat{r} + C_{Y\delta_a}\delta_a + C_{Y\delta_r}\delta_r
$$

---

## 12. Aerodynamic Moment Model

The aerodynamic moments are:

$$
L = \bar{q} S b C_l
$$

$$
M = \bar{q} S \bar{c} C_m
$$

$$
N = \bar{q} S b C_n
$$

The rolling-moment coefficient is:

$$
C_l = C_{l\beta}\beta + C_{l\hat{p}}\hat{p} + C_{l\hat{r}}\hat{r} + C_{l\delta_a}\delta_a + C_{l\delta_r}\delta_r
$$

The pitching-moment coefficient is:

$$
C_m = C_{m0} + C_{m\alpha}\alpha + C_{m\hat{q}}\hat{q} + C_{m\delta_e}\delta_e + C_{m\delta_{tv}}\delta_{tv}
$$

The yawing-moment coefficient is:

$$
C_n = C_{n\beta}\beta + C_{n\hat{p}}\hat{p} + C_{n\hat{r}}\hat{r} + C_{n\delta_a}\delta_a + C_{n\delta_r}\delta_r
$$

The signs of these derivatives should be chosen to produce physically reasonable aircraft behavior:

| Stability derivative | Required sign | Physical meaning |
|---|---:|---|
| $C_{m_{\alpha}}$ | $< 0$ | Longitudinal static stability in the simplified model |
| $C_{m_{\hat{q}}}$ | $< 0$ | Pitch-rate damping |
| $C_{l_{\hat{p}}}$ | $< 0$ | Roll-rate damping |
| $C_{n_{\hat{r}}}$ | $< 0$ | Yaw-rate damping |
| $C_{n_{\beta}}$ | $> 0$ | Directional weathercock stability |

A real high-performance fighter may be relaxed-stability or flight-control-augmented. This simplified model should therefore be interpreted as an educational surrogate, not the actual aircraft.

---

## 13. Propulsion and Thrust Vectoring Model

The propulsive force is modeled as:

$$
T = \delta_T T_{max}
$$

where:

- \(\delta_T\): normalized throttle command between 0 and 1.
- \(T_{max}\): maximum total available thrust.

The total body-axis force is:

$$
X = X_{aero} + T\cos\delta_{tv}
$$

$$
Z = Z_{aero} - T\sin\delta_{tv}
$$

For a simplified pitch thrust-vectoring model, the thrust-vectoring contribution to pitching moment can be approximated as:

$$
M_{tv} = T l_{tv}\sin\delta_{tv}
$$

where \(l_{tv}\) is an effective moment arm between the thrust-vectoring force line and the aircraft center of gravity.

The total pitching moment becomes:

$$
M = M_{aero} + M_{tv}
$$

In the first project version, thrust vectoring may also be represented directly through a coefficient term \(C_{m\delta_{tv}}\delta_{tv}\). This is simpler and easier to use during early controller design.

---

## 14. Representative F-22-Inspired Parameters

The following parameters are used only as a representative educational model:

| Parameter | Symbol | Representative Value | Unit |
|---|---:|---:|---:|
| Mass | \(m\) | 29000 | kg |
| Wing reference area | \(S\) | 78.0 | m² |
| Wingspan | \(b\) | 13.6 | m |
| Mean aerodynamic chord | \(\bar{c}\) | 5.2 | m |
| Roll inertia | \(I_x\) | 250000 | kg·m² |
| Pitch inertia | \(I_y\) | 1200000 | kg·m² |
| Yaw inertia | \(I_z\) | 1400000 | kg·m² |
| Product of inertia | \(I_{xz}\) | 20000 | kg·m² |
| Maximum thrust | \(T_{max}\) | 310000 | N |
| Gravity | \(g\) | 9.80665 | m/s² |

These values are not proprietary F-22 values. They are representative values selected to create a plausible high-performance fighter simulation.

---

## 15. Trim Condition

Before designing a linear controller, the nonlinear aircraft must be trimmed around a steady flight condition.

For steady, wings-level flight:

$$
\dot{x} = 0
$$

A simplified trim condition may use:

$$
p = q = r = 0
$$

$$
\phi = 0
$$

$$
\beta = 0
$$

$$
\theta \approx \alpha
$$

The trim variables may include:

$$
z_{trim} = \begin{bmatrix} \alpha & \delta_e & \delta_T & \delta_{tv} \end{bmatrix}^T
$$

The trim solver adjusts these variables until the residual equations are minimized:

$$
F_{trim}(z_{trim}) = 0
$$

Typical residuals include:

- Longitudinal acceleration residual.
- Vertical acceleration residual.
- Pitch acceleration residual.
- Desired airspeed condition.

A numerical optimizer such as `fsolve` or `fmincon` can be used in MATLAB.

---

## 16. Numerical Linearization

After trim, the nonlinear model is linearized around the trim state and trim input.

The nonlinear model is:

$$
\dot{x} = f(x,u)
$$

The linear perturbation model is:

$$
\Delta\dot{x} = A\Delta x + B\Delta u
$$

where:

$$
A = \left.\frac{\partial f}{\partial x}\right|_{x_0,u_0}
$$

$$
B = \left.\frac{\partial f}{\partial u}\right|_{x_0,u_0}
$$

The finite-difference approximation for the \(A\) matrix is:

$$
A_i \approx \frac{f(x_0 + \epsilon e_i,u_0) - f(x_0 - \epsilon e_i,u_0)}{2\epsilon}
$$

The finite-difference approximation for the \(B\) matrix is:

$$
B_j \approx \frac{f(x_0,u_0 + \epsilon e_j) - f(x_0,u_0 - \epsilon e_j)}{2\epsilon}
$$

For the first MIMO controller, the reduced control-design state is:

$$
x_c = \begin{bmatrix} u & v & w & p & q & r & \phi & \theta & \psi \end{bmatrix}^T
$$

The Earth-frame position states are kept for trajectory plotting but are not required for the first attitude-stabilization LQR controller.

---

## 17. MIMO State-Space Model

The linearized MIMO system is:

$$
\Delta\dot{x}_c = A_c\Delta x_c + B_c\Delta u_c
$$

where:

$$
\Delta x_c = x_c - x_{c,trim}
$$

and:

$$
\Delta u_c = u_c - u_{c,trim}
$$

The output equation is:

$$
y = C\Delta x_c + D\Delta u_c
$$

For full-state feedback, the controller assumes that all reduced states are available. In a more realistic later phase, the states would be estimated using an observer or Kalman filter.

---

## 18. Controllability and Observability

The controllability matrix is:

$$
\mathcal{C} = \begin{bmatrix} B_c & A_cB_c & A_c^2B_c & \cdots & A_c^{n-1}B_c \end{bmatrix}
$$

The system is controllable if:

$$
\text{rank}(\mathcal{C}) = n
$$

where \(n\) is the number of states.

The observability matrix is:

$$
\mathcal{O} = \begin{bmatrix} C \\ CA_c \\ CA_c^2 \\ \vdots \\ CA_c^{n-1} \end{bmatrix}
$$

The system is observable if:

$$
\text{rank}(\mathcal{O}) = n
$$

For this project, controllability is especially important because the aircraft uses multiple effectors that influence coupled roll, pitch, and yaw dynamics.

---

## 19. LQR Control Design

The Linear Quadratic Regulator minimizes the cost function:

$$
J = \int_0^\infty \left(\Delta x^T Q \Delta x + \Delta u^T R \Delta u\right)dt
$$

The control law is:

$$
\Delta u = -K\Delta x
$$

Therefore, the commanded control input is:

$$
u_c = u_{trim} - K(x_c - x_{c,trim})
$$

The closed-loop system is:

$$
\Delta\dot{x}_c = (A_c - B_cK)\Delta x_c
$$

The closed-loop poles are the eigenvalues of:

$$
A_{cl} = A_c - B_cK
$$

---

## 20. Selection of Q and R Using Bryson's Rule

Bryson's Rule provides a physically meaningful way to choose diagonal LQR weights.

For each state:

$$
Q_{ii} = \frac{1}{x_{i,max}^2}
$$

For each control input:

$$
R_{jj} = \frac{1}{u_{j,max}^2}
$$

Recommended maximum state deviations for the first fighter-style MIMO controller are:

| State | Maximum Deviation |
|---|---:|
| \(u\) | 20 m/s |
| \(v\) | 10 m/s |
| \(w\) | 10 m/s |
| \(p\) | 60 deg/s |
| \(q\) | 60 deg/s |
| \(r\) | 45 deg/s |
| \(\phi\) | 45 deg |
| \(\theta\) | 30 deg |
| \(\psi\) | 30 deg |

Recommended maximum control deviations are:

| Input | Maximum Deviation |
|---|---:|
| \(\delta_e\) | 25 deg |
| \(\delta_a\) | 25 deg |
| \(\delta_r\) | 30 deg |
| \(\delta_T\) | 0.30 normalized throttle |
| \(\delta_{tv}\) | 20 deg |

The MATLAB implementation is:

```matlab
u_max     = 20;
v_max     = 10;
w_max     = 10;
p_max     = deg2rad(60);
q_max     = deg2rad(60);
r_max     = deg2rad(45);
phi_max   = deg2rad(45);
theta_max = deg2rad(30);
psi_max   = deg2rad(30);

de_max  = deg2rad(25);
da_max  = deg2rad(25);
dr_max  = deg2rad(30);
dT_max  = 0.30;
dtv_max = deg2rad(20);

Q = diag([ ...
    1/u_max^2, ...
    1/v_max^2, ...
    1/w_max^2, ...
    1/p_max^2, ...
    1/q_max^2, ...
    1/r_max^2, ...
    1/phi_max^2, ...
    1/theta_max^2, ...
    1/psi_max^2]);

R = diag([ ...
    1/de_max^2, ...
    1/da_max^2, ...
    1/dr_max^2, ...
    1/dT_max^2, ...
    1/dtv_max^2]);

K = lqr(Ac,Bc,Q,R);
```

---

## 21. Control Saturation

The controller output must be limited to physically meaningful actuator bounds:

$$
\delta_e \in [-25^\circ, 25^\circ]
$$

$$
\delta_a \in [-25^\circ, 25^\circ]
$$

$$
\delta_r \in [-30^\circ, 30^\circ]
$$

$$
\delta_T \in [0, 1]
$$

$$
\delta_{tv} \in [-20^\circ, 20^\circ]
$$

Actuator saturation prevents the nonlinear simulation from using unrealistic control commands.

---

## 22. Open-Loop and Closed-Loop Analysis

The open-loop system matrix is:

$$
A_c
$$

The closed-loop system matrix is:

$$
A_{cl} = A_c - B_cK
$$

The open-loop and closed-loop eigenvalues are compared using a pole map.

A successful controller should:

1. Move unstable or weakly damped poles into the left-half plane.
2. Increase damping of oscillatory modes.
3. Avoid excessively fast poles that would demand unrealistic actuator motion.
4. Reduce roll, pitch, and yaw rates after disturbance.
5. Keep elevator, aileron, rudder, throttle, and thrust-vectoring commands within limits.

---

## 23. Expected Simulation Outputs

The project should generate the following plots:

1. Roll angle response, \(\phi(t)\).
2. Pitch angle response, \(\theta(t)\).
3. Yaw angle response, \(\psi(t)\).
4. Angular rates, \(p(t)\), \(q(t)\), and \(r(t)\).
5. Body-axis velocities, \(u(t)\), \(v(t)\), and \(w(t)\).
6. Control inputs: elevator, aileron, rudder, throttle, and thrust vectoring.
7. Open-loop versus closed-loop pole map.
8. 3D flight trajectory.
9. Angle of attack and sideslip response.
10. Control saturation indicators.

These plots demonstrate both the dynamic behavior and the control effort required for stabilization.

---

## 24. Validation Approach

The model should be validated through engineering consistency checks rather than comparison to real F-22 flight data.

Recommended checks:

1. Verify that forces and moments have correct units.
2. Verify that trim residuals are near zero.
3. Confirm that the open-loop nonlinear simulation behaves reasonably near trim.
4. Confirm that the numerical linearization produces finite, well-scaled \(A\) and \(B\) matrices.
5. Check controllability rank.
6. Check closed-loop eigenvalues.
7. Confirm that the closed-loop nonlinear response stabilizes after a perturbation.
8. Confirm that control inputs stay within actuator limits.
9. Compare linear closed-loop response against nonlinear closed-loop response for small perturbations.
10. Document any limitations or unrealistic assumptions.

---

## 25. Limitations

This model has important limitations:

1. The aerodynamic coefficients are simplified and not based on a real F-22 aerodynamic database.
2. The mass and inertia values are representative approximations.
3. Compressibility effects are not modeled in the first version.
4. Supersonic aerodynamics are not modeled.
5. Control-surface actuator dynamics are initially neglected.
6. Sensor dynamics and state-estimation errors are initially neglected.
7. Engine spool dynamics are simplified.
8. Thrust-vectoring dynamics are simplified.
9. Euler angles are used instead of quaternions.
10. The controller is initially fixed-gain and not gain-scheduled.

These limitations should be stated clearly in the project README and documentation.

---

## 26. Future Improvements

Future project extensions may include:

1. Quaternion-based attitude representation.
2. Actuator rate limits and actuator dynamics.
3. Sensor noise and Kalman-filter state estimation.
4. Gain-scheduled LQR across multiple flight conditions.
5. Nonlinear dynamic inversion.
6. Control allocation between aerodynamic surfaces and thrust vectoring.
7. Gust and turbulence modeling.
8. Mach-dependent aerodynamic coefficient tables.
9. Automatic landing guidance using the nonlinear fighter model.
10. Simulink implementation using Aerospace Blockset.

---

## 27. Engineering Interpretation

This project demonstrates that a high-performance fighter aircraft is a strongly coupled multivariable system. Elevator, aileron, rudder, throttle, and thrust vectoring do not affect only one axis independently. Instead, each effector can influence multiple states through aerodynamic, inertial, and propulsive coupling.

The MIMO LQR controller is useful because it computes a coordinated feedback law using the full linearized system model. Instead of designing separate single-input single-output loops for pitch, roll, and yaw, LQR designs a single gain matrix that accounts for the interaction between all selected states and control inputs.

This makes the project a natural progression from earlier longitudinal pitch-damper and altitude-hold work into full aircraft flight-control design.

---

## 28. Portfolio Summary

This theory document supports the following GitHub project:

```text
phase4_5_f22_inspired_mimo_6dof/
```

The project demonstrates:

- Nonlinear 6-DOF rigid-body aircraft modeling.
- Fighter-style aerodynamic force and moment modeling.
- Multiple-input multiple-output state-space control.
- Trim and numerical linearization.
- LQR controller design using Bryson's Rule.
- Closed-loop nonlinear simulation.
- Professional engineering documentation.

This project should be presented as an educational, F-22-inspired simulation and not as a reproduction of the real F-22 aircraft or its flight-control system.

---

## 29. Public Reference Basis

The F-22-inspired project uses public facts only for general aircraft context, such as approximate wingspan, length, engine class, and the existence of two-dimensional thrust-vectoring nozzles. Publicly available references include the U.S. Air Force F-22 Raptor fact sheet.

All aerodynamic derivatives, inertia values, control-law structures, and simulation parameters in this project are representative educational assumptions.
