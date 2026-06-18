# Aerodynamic Model

## Project Context

This document defines the simplified aerodynamic coefficient model used for the F-22-inspired MIMO 6-DOF fighter aircraft simulation.

The model is intended for educational flight-dynamics, control-design, and portfolio demonstration purposes. It is not a representation of the real F-22 aerodynamic database, flight-control laws, or proprietary aircraft model.

The aerodynamic model converts aircraft states and control inputs into nondimensional force and moment coefficients. These coefficients are then converted into body-axis aerodynamic forces and moments for use in the nonlinear 6-DOF equations of motion.

---

## 1. Body-Axis Convention

The aircraft body frame is fixed to the aircraft center of gravity.

| Axis | Positive direction |
|---|---|
| $x_b$ | Forward through the nose |
| $y_b$ | Out the right wing |
| $z_b$ | Downward |

The body-axis force components are:

- $X$: axial force along the body $x_b$-axis
- $Y$: side force along the body $y_b$-axis
- $Z$: normal force along the body $z_b$-axis

The body-axis moment components are:

- $L$: rolling moment about the body $x_b$-axis
- $M$: pitching moment about the body $y_b$-axis
- $N$: yawing moment about the body $z_b$-axis

---

## 2. Airspeed and Aerodynamic Angles

The total airspeed is:

$$
V = \sqrt{u^2 + v^2 + w^2}
$$

The angle of attack is:

$$
\alpha = \tan^{-1}\left(\frac{w}{u}\right)
$$

In MATLAB, the more robust implementation is:

```matlab
alpha = atan2(w,u);
```

The sideslip angle is:

$$
\beta = \sin^{-1}\left(\frac{v}{V}\right)
$$

For numerical robustness, the implementation should protect against division by zero:

```matlab
V = sqrt(u^2 + v^2 + w^2);
V = max(V,1e-3);

alpha = atan2(w,u);
beta = asin(max(min(v/V,1),-1));
```

---

## 3. Dynamic Pressure

Dynamic pressure is:

$$
\bar{q} = \frac{1}{2}\rho V^2
$$

where:

- $\rho$ is air density
- $V$ is true airspeed

For the first simulation version, a constant sea-level density may be used:

$$
\rho = 1.225\,\text{kg/m}^3
$$

A later version can replace this with a standard-atmosphere model:

$$
\rho = \rho(h)
$$

---

## 4. Nondimensional Angular Rates

Aerodynamic damping derivatives are normally expressed using nondimensional angular rates.

The nondimensional roll-rate term is:

$$
\hat{p} = \frac{p b}{2V}
$$

The nondimensional pitch-rate term is:

$$
\hat{q} = \frac{q \bar{c}}{2V}
$$

The nondimensional yaw-rate term is:

$$
\hat{r} = \frac{r b}{2V}
$$

where:

- $p$, $q$, and $r$ are body-axis angular rates
- $b$ is wingspan
- $\bar{c}$ is mean aerodynamic chord
- $V$ is true airspeed

---

## 5. Control Inputs

The simplified aerodynamic model uses the following control inputs:

| Input | Symbol | Unit | Description |
|---|---:|---:|---|
| Elevator / stabilator | $\delta_e$ | rad | Longitudinal control input |
| Aileron | $\delta_a$ | rad | Roll control input |
| Rudder | $\delta_r$ | rad | Yaw control input |
| Pitch thrust vectoring | $\delta_{tv}$ | rad | Simplified pitch thrust-vectoring input |

The normalized throttle command $\delta_T$ is treated in the propulsion model rather than the aerodynamic coefficient model.

---

## 6. Force Coefficients

The body-axis aerodynamic force coefficients are:

- $C_X$: axial-force coefficient
- $C_Y$: side-force coefficient
- $C_Z$: normal-force coefficient

These coefficients are used to calculate aerodynamic forces along the aircraft body axes.

---

## 6.1 Axial-Force Coefficient

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

## 6.2 Side-Force Coefficient

The side-force coefficient includes contributions from sideslip angle, nondimensional roll rate, nondimensional yaw rate, aileron deflection, and rudder deflection.

$$
C_Y =
C_{Y\beta}\beta
+ C_{Y\hat{p}}\hat{p}
+ C_{Y\hat{r}}\hat{r}
+ C_{Y\delta_a}\delta_a
+ C_{Y\delta_r}\delta_r
$$

For a conventional aircraft model, a negative $C_{Y\beta}$ is commonly used so that positive sideslip produces a restoring side-force tendency, depending on the selected body-axis convention.

---

## 6.3 Normal-Force Coefficient

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

With the body $z_b$-axis positive downward, lift usually contributes a negative $C_Z$ value during positive-lift flight.

---

## 7. Moment Coefficients

The body-axis moment coefficients are:

- $C_l$: rolling-moment coefficient
- $C_m$: pitching-moment coefficient
- $C_n$: yawing-moment coefficient

These coefficients are used to calculate aerodynamic moments about the aircraft center of gravity.

---

## 7.1 Rolling-Moment Coefficient

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

## 7.2 Pitching-Moment Coefficient

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

For an F-22-inspired model, pitch thrust vectoring is represented by the term:

$$
C_{m\delta_{tv}}\delta_{tv}
$$

This is a simplified educational representation of thrust-vectoring influence on pitching moment.

---

## 7.3 Yawing-Moment Coefficient

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

## 8. Aerodynamic Forces

The aerodynamic forces are computed from the force coefficients as follows:

$$
X_A = \bar{q} S C_X
$$

$$
Y_A = \bar{q} S C_Y
$$

$$
Z_A = \bar{q} S C_Z
$$

where:

- $S$ is wing reference area
- $\bar{q}$ is dynamic pressure
- $C_X$, $C_Y$, and $C_Z$ are body-axis force coefficients

---

## 9. Aerodynamic Moments

The aerodynamic moments are computed from the moment coefficients as follows:

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

- $b$ is wingspan
- $\bar{c}$ is mean aerodynamic chord
- $C_l$, $C_m$, and $C_n$ are body-axis moment coefficients

---

## 10. Recommended Sign Conventions

The following signs are recommended for physically reasonable first-pass behavior:

| Derivative | Recommended sign | Meaning |
|---|---:|---|
| $C_{m\alpha}$ | $< 0$ | Longitudinal static stability |
| $C_{m\hat{q}}$ | $< 0$ | Pitch-rate damping |
| $C_{l\hat{p}}$ | $< 0$ | Roll-rate damping |
| $C_{n\hat{r}}$ | $< 0$ | Yaw-rate damping |
| $C_{n\beta}$ | $> 0$ | Directional weathercock stability |

These signs are modeling choices for the simplified educational aircraft model and may vary depending on the exact body-axis convention and coefficient definitions used.

---

## 11. MATLAB Implementation Form

A compact MATLAB implementation may use:

```matlab
CX = CX0 + CX_alpha*alpha + CX_alpha2*alpha^2 + CX_de*de;

CY = CY_beta*beta + CY_phat*phat + CY_rhat*rhat + CY_da*da + CY_dr*dr;

CZ = CZ0 + CZ_alpha*alpha + CZ_qhat*qhat + CZ_de*de;

Cl = Cl_beta*beta + Cl_phat*phat + Cl_rhat*rhat + Cl_da*da + Cl_dr*dr;

Cm = Cm0 + Cm_alpha*alpha + Cm_qhat*qhat + Cm_de*de + Cm_dtv*dtv;

Cn = Cn_beta*beta + Cn_phat*phat + Cn_rhat*rhat + Cn_da*da + Cn_dr*dr;
```

The aerodynamic forces and moments can then be computed using:

```matlab
XA = qbar*S*CX;
YA = qbar*S*CY;
ZA = qbar*S*CZ;

LA = qbar*S*b*Cl;
MA = qbar*S*cbar*Cm;
NA = qbar*S*b*Cn;
```

---

## 12. Model Limitations

This aerodynamic model is intentionally simple. It does not include:

- Mach-number dependence
- Reynolds-number dependence
- nonlinear stall behavior
- high-angle-of-attack vortex effects
- control-surface saturation
- actuator dynamics
- aeroelastic effects
- real F-22 aerodynamic lookup tables
- real F-22 control allocation laws

The model should be treated as a first-pass educational coefficient model suitable for testing nonlinear 6-DOF simulation and MIMO control design.

---

## 13. Future Improvements

Future versions can improve the model by adding:

- lookup tables in $\alpha$ and $\beta$
- Mach-dependent coefficient corrections
- nonlinear high-angle-of-attack behavior
- actuator limits and rate limits
- thrust-vectoring control allocation
- trim optimization
- validation against a known open-source aircraft dataset
- comparison between linearized and nonlinear aerodynamic response

