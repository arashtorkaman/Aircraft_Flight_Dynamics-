# Aerodynamic Model

## Project Context

This document defines the simplified aerodynamic model used in the F-22-inspired MIMO 6-DOF fighter aircraft simulation.

The model is intended for educational flight-dynamics, control-design, and simulation purposes. It is not a representation of the actual F-22 aerodynamic database, proprietary control laws, or classified flight-control system.

The aerodynamic model converts aircraft states and control inputs into body-axis aerodynamic forces and moments. These forces and moments are then used by the nonlinear 6-DOF equations of motion.

---

## 1. Purpose of the Aerodynamic Model

The aerodynamic model provides the force and moment coefficients required by the aircraft equations of motion.

The model computes:

- axial-force coefficient, $C_X$
- side-force coefficient, $C_Y$
- normal-force coefficient, $C_Z$
- rolling-moment coefficient, $C_l$
- pitching-moment coefficient, $C_m$
- yawing-moment coefficient, $C_n$

The corresponding aerodynamic forces and moments are:

$$
X_A = \bar{q} S C_X
$$

$$
Y_A = \bar{q} S C_Y
$$

$$
Z_A = \bar{q} S C_Z
$$

$$
L_A = \bar{q} S b C_l
$$

$$
M_A = \bar{q} S \bar{c} C_m
$$

$$
N_A = \bar{q} S b C_n
$$

where $S$ is the wing reference area, $b$ is the wingspan, and $\bar{c}$ is the mean aerodynamic chord.

---

## 2. State and Control Variables

The aerodynamic model uses the following aircraft variables:

| Symbol | Description | Unit |
|---|---|---|
| $u$ | body-axis forward velocity | m/s |
| $v$ | body-axis lateral velocity | m/s |
| $w$ | body-axis vertical velocity | m/s |
| $p$ | roll rate | rad/s |
| $q$ | pitch rate | rad/s |
| $r$ | yaw rate | rad/s |
| $\alpha$ | angle of attack | rad |
| $\beta$ | sideslip angle | rad |
| $V$ | total airspeed | m/s |

The control inputs are:

| Symbol | Description | Unit |
|---|---|---|
| $\delta_e$ | elevator or stabilator deflection | rad |
| $\delta_a$ | aileron deflection | rad |
| $\delta_r$ | rudder deflection | rad |
| $\delta_T$ | normalized throttle command | nondimensional |
| $\delta_{tv}$ | pitch thrust-vectoring deflection | rad |

---

## 3. Airspeed, Angle of Attack, and Sideslip

The total airspeed is:

$$
V = \sqrt{u^2 + v^2 + w^2}
$$

The angle of attack is:

$$
\alpha = \tan^{-1}\left(\frac{w}{u}\right)
$$

In simulation, the numerically safer implementation is:

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

where $\rho$ is air density.

For the first implementation, constant sea-level density may be used:

$$
\rho = 1.225 \ \mathrm{kg/m^3}
$$

A later version may replace constant density with a standard-atmosphere model:

$$
\rho = \rho(h)
$$

---

## 5. Nondimensional Angular Rates

The aerodynamic damping derivatives use nondimensional angular rates.

The nondimensional roll-rate term is:

$$
\hat{p} = \frac{pb}{2V}
$$

The nondimensional pitch-rate term is:

$$
\hat{q} = \frac{q\bar{c}}{2V}
$$

The nondimensional yaw-rate term is:

$$
\hat{r} = \frac{rb}{2V}
$$

These nondimensional terms allow the aerodynamic damping derivatives to scale consistently with flight speed and aircraft geometry.

---

## 6. Force Coefficients

The body-axis aerodynamic force coefficients are:

- $C_X$: axial-force coefficient
- $C_Y$: side-force coefficient
- $C_Z$: normal-force coefficient

The body frame uses $x_b$ positive forward, $y_b$ positive out the right wing, and $z_b$ positive downward.

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

For a conventional aircraft model, a negative $C_{Y\beta}$ is commonly used so that positive sideslip produces a restoring side-force tendency, depending on the selected body-axis convention.

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

With the body $z_b$-axis positive downward, lift usually contributes a negative $C_Z$ value during positive-lift flight.

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

## 8. Lift and Drag Approximation

For early implementation, lift and drag may be modeled first, then converted into body-axis axial and normal force coefficients.

A simplified lift model is:

$$
C_L =
C_{L0}
+ C_{L\alpha}\alpha
+ C_{L\hat{q}}\hat{q}
+ C_{L\delta_e}\delta_e
$$

A simplified drag model is:

$$
C_D = C_{D0} + K C_L^2
$$

The body-axis conversion is:

$$
C_X = -C_D\cos\alpha + C_L\sin\alpha
$$

$$
C_Z = -C_D\sin\alpha - C_L\cos\alpha
$$

This representation is useful because lift and drag are intuitive, while the 6-DOF equations require body-axis force components.

---

## 9. Aerodynamic Forces and Moments

The aerodynamic forces are:

$$
X_A = \bar{q} S C_X
$$

$$
Y_A = \bar{q} S C_Y
$$

$$
Z_A = \bar{q} S C_Z
$$

The aerodynamic moments are:

$$
L_A = \bar{q} S b C_l
$$

$$
M_A = \bar{q} S \bar{c} C_m
$$

$$
N_A = \bar{q} S b C_n
$$

These forces and moments are then passed into the nonlinear aircraft equations of motion.

---

## 10. Stability-Derivative Sign Summary

The following signs produce physically reasonable first-order damping and static-stability behavior for the simplified model.

| Derivative | Recommended sign | Meaning |
|---|---:|---|
| $C_{m\alpha}$ | $< 0$ | longitudinal static stability |
| $C_{m\hat{q}}$ | $< 0$ | pitch-rate damping |
| $C_{l\hat{p}}$ | $< 0$ | roll-rate damping |
| $C_{n\hat{r}}$ | $< 0$ | yaw-rate damping |
| $C_{n\beta}$ | $> 0$ | directional weathercock stability |

The side-force derivative $C_{Y\beta}$ depends on the selected body-axis and sideslip sign convention. The convention should be stated clearly before interpreting the sign physically.

---

## 11. MATLAB Implementation Notes

A MATLAB implementation should compute the aerodynamic variables in the following order:

```matlab
V = sqrt(u^2 + v^2 + w^2);
V = max(V,1e-3);

alpha = atan2(w,u);
beta  = asin(max(min(v/V,1),-1));

qbar = 0.5*rho*V^2;

p_hat = p*b/(2*V);
q_hat = q*cbar/(2*V);
r_hat = r*b/(2*V);
```

Then the coefficient model may be evaluated:

```matlab
CX = CX0 + CX_alpha*alpha + CX_alpha2*alpha^2 + CX_de*de;
CY = CY_beta*beta + CY_phat*p_hat + CY_rhat*r_hat + CY_da*da + CY_dr*dr;
CZ = CZ0 + CZ_alpha*alpha + CZ_qhat*q_hat + CZ_de*de;

Cl = Cl_beta*beta + Cl_phat*p_hat + Cl_rhat*r_hat + Cl_da*da + Cl_dr*dr;
Cm = Cm0 + Cm_alpha*alpha + Cm_qhat*q_hat + Cm_de*de + Cm_dtv*dtv;
Cn = Cn_beta*beta + Cn_phat*p_hat + Cn_rhat*r_hat + Cn_da*da + Cn_dr*dr;
```

Finally, forces and moments are computed:

```matlab
XA = qbar*S*CX;
YA = qbar*S*CY;
ZA = qbar*S*CZ;

LA = qbar*S*b*Cl;
MA = qbar*S*cbar*Cm;
NA = qbar*S*b*Cn;
```

---

## 12. Limitations

This aerodynamic model is intentionally simplified. It does not include:

- Mach-number dependence
- Reynolds-number dependence
- compressibility effects
- stall or post-stall aerodynamics
- aerodynamic hysteresis
- control-surface rate limits
- actuator dynamics
- detailed thrust-vectoring nozzle dynamics
- full aerodynamic lookup tables
- proprietary F-22 aerodynamic data

For the portfolio project, this model is sufficient for demonstrating nonlinear simulation, numerical linearization, MIMO control design, and closed-loop flight-dynamics analysis.

---

## 13. Future Improvements

Future versions may add:

- aerodynamic lookup tables indexed by $\alpha$, $\beta$, and Mach number
- actuator saturation and rate limits
- gain scheduling over airspeed and altitude
- engine and thrust-vectoring actuator dynamics
- wind and turbulence models
- automatic landing guidance laws
- nonlinear control allocation
- validation against a known open-source aircraft model

---

## 14. Summary

This document defines a simplified coefficient-based aerodynamic model for the F-22-inspired MIMO 6-DOF fighter aircraft simulation.

The model uses body-axis force and moment coefficients, nondimensional angular rates, dynamic pressure, and control-surface derivatives to generate aerodynamic forces and moments for the nonlinear equations of motion.

The model is not a real F-22 aerodynamic database. It is an educational and portfolio-oriented approximation designed to demonstrate the engineering workflow behind aircraft simulation and multivariable flight-control design.
