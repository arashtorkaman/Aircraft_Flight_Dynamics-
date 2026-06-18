# Sections 6 and 7 — Aerodynamic Force and Moment Coefficients

This file contains corrected GitHub-safe Markdown/LaTeX for Sections 6 and 7 of the aerodynamic model document.

The notation uses comma-separated coefficient subscripts, such as $C_{X,\alpha}$ and $C_{l,\hat{p}}$, because this is clearer and more robust than nested-subscript notation.

---

## 6. Body-Axis Force Coefficients

The body-axis aerodynamic force coefficients are:

- $C_X$: axial-force coefficient
- $C_Y$: side-force coefficient
- $C_Z$: normal-force coefficient

These coefficients are used to compute the aerodynamic forces along the body axes of the aircraft.

---

### 6.1 Axial-Force Coefficient

The axial-force coefficient is modeled as:

$$
C_X =
C_{X0}
+ C_{X,\alpha}\alpha
+ C_{X,\alpha^2}\alpha^2
+ C_{X,\delta_e}\delta_e
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
C_{Y,\beta}\beta
+ C_{Y,\hat{p}}\hat{p}
+ C_{Y,\hat{r}}\hat{r}
+ C_{Y,\delta_a}\delta_a
+ C_{Y,\delta_r}\delta_r
$$

For a conventional aircraft model, a negative $C_{Y,\beta}$ is commonly used so that positive sideslip produces a restoring side-force tendency, depending on the selected body-axis convention.

---

### 6.3 Normal-Force Coefficient

The normal-force coefficient is modeled as:

$$
C_Z =
C_{Z0}
+ C_{Z,\alpha}\alpha
+ C_{Z,\hat{q}}\hat{q}
+ C_{Z,\delta_e}\delta_e
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
C_{l,\beta}\beta
+ C_{l,\hat{p}}\hat{p}
+ C_{l,\hat{r}}\hat{r}
+ C_{l,\delta_a}\delta_a
+ C_{l,\delta_r}\delta_r
$$

The term $C_{l,\hat{p}}\hat{p}$ represents roll-rate damping. For physically reasonable roll damping, the derivative should usually satisfy:

$$
C_{l,\hat{p}} < 0
$$

---

### 7.2 Pitching-Moment Coefficient

The pitching-moment coefficient is modeled as:

$$
C_m =
C_{m0}
+ C_{m,\alpha}\alpha
+ C_{m,\hat{q}}\hat{q}
+ C_{m,\delta_e}\delta_e
+ C_{m,\delta_{tv}}\delta_{tv}
$$

The derivative $C_{m,\alpha}$ controls the static pitch stability of the simplified model. For a statically stable conventional aircraft model, the derivative should satisfy:

$$
C_{m,\alpha} < 0
$$

The derivative $C_{m,\hat{q}}$ provides pitch-rate damping. For physically reasonable pitch damping:

$$
C_{m,\hat{q}} < 0
$$

For an F-22-inspired model, pitch thrust vectoring is represented by the term:

$$
C_{m,\delta_{tv}}\delta_{tv}
$$

This is a simplified educational representation of thrust-vectoring influence on pitching moment.

---

### 7.3 Yawing-Moment Coefficient

The yawing-moment coefficient is modeled as:

$$
C_n =
C_{n,\beta}\beta
+ C_{n,\hat{p}}\hat{p}
+ C_{n,\hat{r}}\hat{r}
+ C_{n,\delta_a}\delta_a
+ C_{n,\delta_r}\delta_r
$$

The derivative $C_{n,\beta}$ represents directional static stability. For weathercock stability, the simplified model should use:

$$
C_{n,\beta} > 0
$$

The derivative $C_{n,\hat{r}}$ provides yaw-rate damping. For physically reasonable yaw damping:

$$
C_{n,\hat{r}} < 0
$$
