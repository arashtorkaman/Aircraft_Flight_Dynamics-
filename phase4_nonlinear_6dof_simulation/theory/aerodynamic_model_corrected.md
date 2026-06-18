# Aerodynamic Model

## Project Context

This document defines the simplified aerodynamic model used in the F-22-inspired MIMO 6-DOF fighter aircraft simulation.

The purpose of this model is to generate aerodynamic body-axis forces and moments for nonlinear simulation, trim analysis, numerical linearization, and MIMO control design. The model is intended for educational and portfolio demonstration purposes. It is not a representation of the actual F-22 aerodynamic database, flight-control system, or proprietary aircraft model.

The aerodynamic model uses coefficient-based equations. These equations relate aircraft motion variables, angular rates, and control-surface deflections to nondimensional aerodynamic force and moment coefficients.

---

## 1. Modeling Purpose

The aerodynamic model provides the forces and moments required by the nonlinear 6-DOF equations of motion.

The model receives:

- Body-axis velocities: $u$, $v$, $w$
- Body-axis angular rates: $p$, $q$, $r$
- Control inputs: $\delta_e$, $\delta_a$, $\delta_r$, $\delta_T$, $\delta_{tv}$
- Aircraft reference parameters: $S$, $b$, $\bar{c}$
- Atmospheric density: $\rho$

The model outputs:

- Body-axis force coefficients: $C_X$, $C_Y$, $C_Z$
- Body-axis moment coefficients: $C_l$, $C_m$, $C_n$
- Aerodynamic forces: $X_A$, $Y_A$, $Z_A$
- Aerodynamic moments: $L_A$, $M_A$, $N_A$

---

## 2. Assumptions

The first implementation uses the following assumptions:

- The aircraft is treated as a rigid body.
- Aerodynamic coefficients are modeled using low-order analytical approximations.
- The aerodynamic database is not based on real F-22 proprietary data.
- Coefficients are representative and chosen for physically reasonable behavior.
- Compressibility effects are neglected in the first implementation.
- Aerodynamic hysteresis, shock effects, buffet, and post-stall behavior are not modeled.
- Control-surface actuator dynamics are neglected in the first implementation.
- Constant air density may be used for initial simulations.
- The model is valid only near moderate angles of attack and sideslip.

---

## 3. Airspeed and Flow Angles

The total airspeed is:

$$
V = \sqrt{u^2 + v^2 + w^2}
$$

The angle of attack is:

$$
\alpha = \tan^{-1}\left(\frac{w}{u}\right)
$$

In software, the recommended implementation is:

```matlab
alpha = atan2(w,u);
```

The sideslip angle is:

$$
\beta = \sin^{-1}\left(\frac{v}{V}\right)
$$

For numerical robustness, the argument of the inverse sine should be limited to the interval $[-1,1]$.

```matlab
V = sqrt(u^2 + v^2 + w^2);
V = max(V,1e-3);
beta = asin(max(min(v/V,1),-1));
```

---

## 4. Dynamic Pressure

Dynamic pressure is defined as:

$$
\bar{q} = \frac{1}{2}\rho V^2
$$

where:

- $\rho$ is air density
- $V$ is true airspeed

In the first implementation, a constant sea-level value may be used:

$$
\rho = 1.225 \; \text{kg/m}^3
$$

A later version may replace this with a standard-atmosphere model:

$$
\rho = \rho(h)
$$

---

## 5. Nondimensional Angular Rates

Aerodynamic damping derivatives are usually expressed using nondimensional angular rates.

The nondimensional roll rate is:

$$
\hat{p} = \frac{pb}{2V}
$$

The nondimensional pitch rate is:

$$
\hat{q} = \frac{q\bar{c}}{2V}
$$

The nondimensional yaw rate is:

$$
\hat{r} = \frac{rb}{2V}
$$

where:

- $p$ is roll rate
- $q$ is pitch rate
- $r$ is yaw rate
- $b$ is wingspan
- $\bar{c}$ is mean aerodynamic chord
- $V$ is true airspeed

---

## 6. Force Coefficients

The body-axis force coefficients are:

- $C_X$: axial-force coefficient
- $C_Y$: side-force coefficient
- $C_Z$: normal-force coefficient

These are defined in the aircraft body frame.

---

### 6.1 Axial-Force Coefficient

The axial-force coefficient is modeled as:

$$
C_X =
C_{X0}
+ C_{X\alpha}\alpha
+ C_{X\alpha^2}\alpha^2
+ C_{X\delta_e}\delta_e
$$

The axial-force coefficient may also be approximated using lift and drag coefficients:

$$
C_X = -C_D\cos\alpha + C_L\sin\alpha
$$

This second form is often convenient when the aerodynamic model is first constructed using lift and drag.

---

### 6.2 Side-Force Coefficient

The side-force coefficient includes contributions from sideslip angle, nondimensional roll rate, nondimensional yaw rate, aileron deflection, and rudder deflection.

$$
C_Y =
C_{Y\beta}\beta
+ C_{Y\hat{p}}\hat{p}
+ C_{Y\hat{r}}\hat{r}
+ C_{Y\delta_a}\delta_a
+ C_{Y\delta_r}\delta_r
$$

For a conventional aircraft, a negative $C_{Y\beta}$ is commonly used so that positive sideslip produces a restoring side force depending on the chosen axis convention.

---

### 6.3 Normal-Force Coefficient

The normal-force coefficient is modeled as:

$$
C_Z =
C_{Z0}
+ C_{Z\alpha}\alpha
+ C_{Z\hat{q}}\hat{q}
+ C_{Z\delta_e}\delta_e
$$

The normal-force coefficient may also be computed from lift and drag coefficients:

$$
C_Z = -C_D\sin\alpha - C_L\cos\alpha
$$

With the body $z$-axis positive downward, lift usually contributes a negative $C_Z$ value during positive lift flight.

---

## 7. Moment Coefficients

The body-axis moment coefficients are:

- $C_l$: rolling-moment coefficient
- $C_m$: pitching-moment coefficient
- $C_n$: yawing-moment coefficient

These coefficients are used to calculate aerodynamic moments about the aircraft center of gravity.

---

### 7.1 Rolling-Moment Coefficient

The rolling-moment coefficient is modeled as:

$$
C_l =
C_{l\beta}\beta
+ C_{l\hat{p}}\hat{p}
+ C_{l\hat{r}}\hat{r}
+ C_{l\delta_a}\delta_a
+ C_{l\delta_r}\delta_r
$$

The term $C_{l\hat{p}}\hat{p}$ represents roll-rate damping. For physically reasonable roll damping, the derivative should usually satisfy:

$$
C_{l\hat{p}} < 0
$$

---

### 7.2 Pitching-Moment Coefficient

The pitching-moment coefficient is modeled as:

$$
C_m =
C_{m0}
+ C_{m\alpha}\alpha
+ C_{m\hat{q}}\hat{q}
+ C_{m\delta_e}\delta_e
+ C_{m\delta_{tv}}\delta_{tv}
$$

The derivative $C_{m\alpha}$ controls the static pitch stability of the simplified model. For a statically stable conventional aircraft model, the derivative should satisfy:

$$
C_{m\alpha} < 0
$$

The derivative $C_{m\hat{q}}$ provides pitch-rate damping. For physically reasonable pitch damping:

$$
C_{m\hat{q}} < 0
$$

For an F-22-inspired model, pitch thrust-vectoring is represented by the term:

$$
C_{m\delta_{tv}}\delta_{tv}
$$

This is a simplified educational representation of thrust-vectoring influence on pitching moment.

---

### 7.3 Yawing-Moment Coefficient

The yawing-moment coefficient is modeled as:

$$
C_n =
C_{n\beta}\beta
+ C_{n\hat{p}}\hat{p}
+ C_{n\hat{r}}\hat{r}
+ C_{n\delta_a}\delta_a
+ C_{n\delta_r}\delta_r
$$

The derivative $C_{n\beta}$ represents directional static stability. For weathercock stability, the simplified model should use:

$$
C_{n\beta} > 0
$$

The derivative $C_{n\hat{r}}$ provides yaw-rate damping. For physically reasonable yaw damping:

$$
C_{n\hat{r}} < 0
$$

---

## 8. Optional Lift and Drag Model

For an alternative implementation, the aerodynamic model may first compute lift and drag coefficients.

The lift coefficient is modeled as:

$$
C_L =
C_{L0}
+ C_{L\alpha}\alpha
+ C_{L\hat{q}}\hat{q}
+ C_{L\delta_e}\delta_e
$$

The drag coefficient is modeled as:

$$
C_D =
C_{D0}
+ K C_L^2
+ C_{D\beta}\beta^2
+ C_{D\delta_e}\delta_e^2
$$

The lift and drag coefficients can then be converted to body-axis coefficients:

$$
C_X = -C_D\cos\alpha + C_L\sin\alpha
$$

$$
C_Z = -C_D\sin\alpha - C_L\cos\alpha
$$

This form is useful for early simulations because it produces intuitive lift and drag behavior.

---

## 9. Aerodynamic Forces

The aerodynamic body-axis forces are calculated from the force coefficients.

The axial force is:

$$
X_A = \bar{q} S C_X
$$

The side force is:

$$
Y_A = \bar{q} S C_Y
$$

The normal force is:

$$
Z_A = \bar{q} S C_Z
$$

where:

- $X_A$ is aerodynamic axial force
- $Y_A$ is aerodynamic side force
- $Z_A$ is aerodynamic normal force
- $S$ is wing reference area

---

## 10. Aerodynamic Moments

The aerodynamic rolling moment is:

$$
L_A = \bar{q} S b C_l
$$

The aerodynamic pitching moment is:

$$
M_A = \bar{q} S \bar{c} C_m
$$

The aerodynamic yawing moment is:

$$
N_A = \bar{q} S b C_n
$$

where:

- $L_A$ is aerodynamic rolling moment
- $M_A$ is aerodynamic pitching moment
- $N_A$ is aerodynamic yawing moment
- $b$ is wingspan
- $\bar{c}$ is mean aerodynamic chord

---

## 11. Total Force and Moment Inputs to the 6-DOF Model

The total body-axis forces used by the nonlinear 6-DOF model are:

$$
X = X_A + X_T
$$

$$
Y = Y_A + Y_T
$$

$$
Z = Z_A + Z_T
$$

The total body-axis moments are:

$$
L = L_A + L_T
$$

$$
M = M_A + M_T
$$

$$
N = N_A + N_T
$$

For the first implementation, lateral and yaw thrust-vectoring components may be neglected:

$$
Y_T = 0
$$

$$
L_T = 0
$$

$$
N_T = 0
$$

Pitch thrust-vectoring may be included through:

$$
M_T = Z_T l_T
$$

where $l_T$ is the thrust-vectoring moment arm.

---

## 12. Thrust and Pitch Thrust Vectoring

The total thrust is modeled as:

$$
T = \delta_T T_{max}
$$

where:

- $\delta_T$ is normalized throttle command
- $T_{max}$ is maximum available thrust

For a simplified pitch thrust-vectoring model:

$$
X_T = T\cos\delta_{tv}
$$

$$
Z_T = -T\sin\delta_{tv}
$$

The thrust-vectoring pitching moment is:

$$
M_T = Z_T l_T
$$

This simplified model captures the first-order effect of pitch thrust vectoring without modeling real nozzle geometry or proprietary flight-control logic.

---

## 13. Stability Derivative Sign Checks

The following sign checks help keep the simplified model physically reasonable.

| Derivative | Recommended sign | Interpretation |
|---|---:|---|
| $C_{m\alpha}$ | $< 0$ | Longitudinal static stability |
| $C_{m\hat{q}}$ | $< 0$ | Pitch-rate damping |
| $C_{l\hat{p}}$ | $< 0$ | Roll-rate damping |
| $C_{n\hat{r}}$ | $< 0$ | Yaw-rate damping |
| $C_{n\beta}$ | $> 0$ | Directional weathercock stability |

These checks are not a complete validation of the aircraft model. They are first-order consistency checks for the simplified aerodynamic coefficient model.

---

## 14. MATLAB Implementation Outline

A basic MATLAB implementation may use the following function structure:

```matlab
function aero = aerodynamic_model(x,u_ctrl,params)

% State extraction
u_body = x(1);
v_body = x(2);
w_body = x(3);
p = x(4);
q = x(5);
r = x(6);

% Control extraction
de  = u_ctrl(1);
da  = u_ctrl(2);
dr  = u_ctrl(3);
dT  = u_ctrl(4);
dtv = u_ctrl(5);

% Airspeed and angles
V = sqrt(u_body^2 + v_body^2 + w_body^2);
V = max(V,1e-3);
alpha = atan2(w_body,u_body);
beta = asin(max(min(v_body/V,1),-1));

% Dynamic pressure
qbar = 0.5*params.rho*V^2;

% Nondimensional rates
p_hat = p*params.b/(2*V);
q_hat = q*params.cbar/(2*V);
r_hat = r*params.b/(2*V);

% Coefficient model
CX = params.CX0 + params.CX_alpha*alpha + params.CX_de*de;
CY = params.CY_beta*beta + params.CY_p*p_hat + params.CY_r*r_hat + params.CY_da*da + params.CY_dr*dr;
CZ = params.CZ0 + params.CZ_alpha*alpha + params.CZ_q*q_hat + params.CZ_de*de;

Cl = params.Cl_beta*beta + params.Cl_p*p_hat + params.Cl_r*r_hat + params.Cl_da*da + params.Cl_dr*dr;
Cm = params.Cm0 + params.Cm_alpha*alpha + params.Cm_q*q_hat + params.Cm_de*de + params.Cm_dtv*dtv;
Cn = params.Cn_beta*beta + params.Cn_p*p_hat + params.Cn_r*r_hat + params.Cn_da*da + params.Cn_dr*dr;

% Aerodynamic forces
XA = qbar*params.S*CX;
YA = qbar*params.S*CY;
ZA = qbar*params.S*CZ;

% Aerodynamic moments
LA = qbar*params.S*params.b*Cl;
MA = qbar*params.S*params.cbar*Cm;
NA = qbar*params.S*params.b*Cn;

% Store outputs
aero.V = V;
aero.alpha = alpha;
aero.beta = beta;
aero.qbar = qbar;
aero.CX = CX;
aero.CY = CY;
aero.CZ = CZ;
aero.Cl = Cl;
aero.Cm = Cm;
aero.Cn = Cn;
aero.XA = XA;
aero.YA = YA;
aero.ZA = ZA;
aero.LA = LA;
aero.MA = MA;
aero.NA = NA;

end
```

---

## 15. Validation Checks

The aerodynamic model should be checked before being used in closed-loop control design.

Recommended checks:

- Verify that $V$ is always positive.
- Verify that $\beta$ remains inside physical limits.
- Verify that $\bar{q}$ increases with $V^2$.
- Verify that positive elevator deflection produces the expected pitching moment sign.
- Verify that positive aileron deflection produces the expected rolling moment sign.
- Verify that positive rudder deflection produces the expected yawing moment sign.
- Verify that damping derivatives oppose angular-rate motion.
- Verify that force and moment units are consistent.
- Verify that the open-loop aircraft response is numerically stable enough for simulation.

---

## 16. Limitations

This aerodynamic model is intentionally simplified.

The model does not include:

- Mach-number dependence
- Reynolds-number dependence
- Supersonic aerodynamic effects
- Stall and post-stall aerodynamics
- Vortex-lift effects
- Control-surface saturation effects inside the aerodynamic database
- Aeroelastic effects
- Real F-22 aerodynamic lookup tables
- Real F-22 control allocation logic
- Real F-22 thrust-vectoring schedules

These limitations should be clearly stated in the project README and documentation.

---

## 17. Future Improvements

Future versions may add:

- Lookup-table aerodynamic coefficients
- Mach-dependent aerodynamic derivatives
- Altitude-dependent atmosphere model
- Control-surface actuator dynamics
- Rate limits and saturation
- Nonlinear lift curve with stall behavior
- Separate left/right stabilator effects
- Separate rudder and flaperon effects
- More realistic thrust-vectoring control allocation
- Trim solver integration
- Linearization at multiple flight conditions
- Gain-scheduled MIMO control

---

## Summary

This aerodynamic model provides a simplified coefficient-based representation of the forces and moments acting on an F-22-inspired fighter aircraft model. It is suitable for educational nonlinear simulation, numerical linearization, and MIMO LQR control design.

The model is not intended to reproduce the actual F-22 aerodynamic database. Its purpose is to demonstrate the correct engineering workflow for building, documenting, and validating a 6-DOF aircraft simulation model for a professional aerospace controls portfolio.
