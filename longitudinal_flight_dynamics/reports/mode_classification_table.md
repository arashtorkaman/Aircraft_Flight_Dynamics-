# Mode Classification Table — Cessna 182 Longitudinal Dynamics (Phase 1)

## 1. Purpose

This file summarizes the longitudinal mode classification for the Cessna 182 cruise-condition state-space model developed in Phase 1.

The model contains the two classical longitudinal modes:

- **short-period mode**
- **phugoid mode**

---

## 2. Open-loop eigenvalues

The open-loop poles of the longitudinal state matrix are:

$$
\lambda_{1,2} = -4.4497 \pm 2.8249i
$$

$$
\lambda_{3,4} = -0.0220 \pm 0.1698i
$$

Because all real parts are negative, the open-loop longitudinal model is stable.

---

## 3. Mode classification table

| Mode | Eigenvalues | Dominant states | Natural frequency $\omega_n$ (rad/s) | Damping ratio $\zeta$ | Approximate period $T$ (s) | Classification |
|---|---|---|---:|---:|---:|---|
| Short-period | $-4.4497 \pm 2.8249i$ | $\alpha,\ q$ | 5.27 | 0.844 | 2.22 | Fast, well-damped oscillatory mode |
| Phugoid | $-0.0220 \pm 0.1698i$ | $u,\ \theta$ | 0.171 | 0.129 | 37.0 | Slow, lightly damped oscillatory mode |

---

## 4. Mode-identification logic

The mode classification is based on the pole locations and the physical states that dominate each motion.

### Short-period mode

The short-period mode is identified as the faster complex pair:

$$
-4.4497 \pm 2.8249i
$$

It is classified as the short-period mode because:

- it has a large negative real part
- it decays quickly
- it has a relatively high oscillation frequency
- it is dominated by $\alpha$ and $q$

### Phugoid mode

The phugoid mode is identified as the slower complex pair:

$$
-0.0220 \pm 0.1698i
$$

It is classified as the phugoid mode because:

- it has a small negative real part
- it decays slowly
- it has a low oscillation frequency
- it is dominated by $u$ and $\theta$

---

## 5. Equations used for classification

For a complex pole pair

$$
s = \sigma \pm j\omega_d
$$

the modal quantities are computed from:

### Natural frequency

$$
\omega_n = \sqrt{\sigma^2 + \omega_d^2}
$$

### Damping ratio

$$
\zeta = \frac{-\sigma}{\omega_n}
$$

### Oscillation period

$$
T = \frac{2\pi}{\omega_d}
$$

These equations were used to compute the entries in the mode classification table.

---

## 6. Engineering interpretation

The open-loop Cessna 182 longitudinal model is dynamically consistent with classical aircraft longitudinal behavior:

- the **short-period mode** represents the rapid pitch and angle-of-attack response
- the **phugoid mode** represents the slow speed-attitude oscillation

This classification supports the use of pitch-rate feedback for closed-loop damping improvement, since the short-period mode is strongly influenced by the pitch-rate state $q$.

---

## 7. Phase 1 summary

The Cessna 182 longitudinal state-space model exhibits two stable oscillatory longitudinal modes:

- a fast, well-damped short-period mode
- a slow, lightly damped phugoid mode

This mode-classification table is part of the Phase 1 stability-analysis documentation.

