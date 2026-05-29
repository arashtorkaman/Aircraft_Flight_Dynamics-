# Root-Locus Design for Pitch Rate Damper

## Phase 2 - Longitudinal Flight Control

---

## 1. Purpose

This document explains the root-locus design procedure used to select the pitch-rate damper gain for the longitudinal aircraft model.

The goal is to determine how the closed-loop poles move as the pitch-rate feedback gain increases.

The design objective is to improve short-period damping while preserving stable aircraft behavior.

---

## 2. Pitch-Rate Damper Control Law

The pitch-rate damper uses the following feedback law:

```math
\delta_e=-K_q q
```

where:

| Symbol | Description |
|---|---|
| `delta_e` | Elevator deflection |
| `K_q` | Pitch-rate feedback gain |
| `q` | Pitch rate |

The gain `K_q` is the design parameter.

Increasing `K_q` changes the closed-loop pole locations and therefore changes the aircraft transient response.

---

## 3. Closed-Loop Matrix

The open-loop model is:

```math
\dot{x}=Ax+B\delta_e
```

Using pitch-rate feedback:

```math
\delta_e=-Kx
```

where:

```math
K=[0\;\;0\;\;K_q\;\;0]
```

The closed-loop matrix is:

```math
A_{cl}=A-BK
```

The closed-loop poles are calculated from:

```math
\lambda_{cl}=eig(A_{cl})
```

---

## 4. Root-Locus Concept

The root locus shows how the system poles move as a feedback gain varies.

For this project, the varying gain is the pitch-rate feedback gain `K_q`.

The root-locus study answers these questions:

- Do the short-period poles move farther left as gain increases?
- Does damping improve?
- Does the system remain stable?
- Does the gain create excessive control activity?
- Is there a practical gain range for the pitch-rate damper?

---

## 5. Design Expectations

For a successful pitch-rate damper:

- Short-period poles should move left in the complex plane.
- The real part of the short-period poles should become more negative.
- The damping ratio should increase.
- The phugoid poles should remain mostly unchanged.
- No poles should move into the right-half plane.

A pole farther left in the complex plane corresponds to faster decay.

A pole closer to the imaginary axis corresponds to slower decay.

A pole in the right-half plane indicates instability.

---

## 6. Gain Sweep Method

The gain is swept over a practical range:

```math
0 \le K_q \le 5
```

For each value of `K_q`:

1. Build the feedback vector.
2. Compute the closed-loop matrix.
3. Compute the eigenvalues.
4. Store the pole locations.
5. Plot the pole migration.

---

## 7. MATLAB Implementation

```matlab
% Gain sweep for pitch-rate damper
Kq_values = 0:0.05:5;

% Storage for closed-loop poles
poles_all = zeros(length(Kq_values), size(A,1));

for i = 1:length(Kq_values)
    Kq = Kq_values(i);
    K = [0 0 Kq 0];
    Acl = A - B*K;
    poles_all(i,:) = eig(Acl).';
end
```

---

## 8. Pole Migration Plot

```matlab
figure;
hold on;
grid on;

for i = 1:size(poles_all,2)
    plot(real(poles_all(:,i)), imag(poles_all(:,i)), 'x-');
end

xlabel('Real Axis');
ylabel('Imaginary Axis');
title('Pitch Rate Damper Root Locus');

xline(0,'k--');
```

---

## 9. Damping Ratio and Natural Frequency

For each complex pole:

```math
\omega_n=\sqrt{\sigma^2+\omega_d^2}
```

```math
\zeta=-\frac{\sigma}{\omega_n}
```

where:

| Symbol | Description |
|---|---|
| `sigma` | Real part of the pole |
| `omega_d` | Imaginary part of the pole |
| `omega_n` | Natural frequency |
| `zeta` | Damping ratio |

The damping ratio should increase for the short-period mode as `K_q` increases.

---

## 10. MATLAB Damping Calculation

```matlab
% Example damping calculation for selected closed-loop matrix
[poles_cl, wn_cl, zeta_cl] = damp(ss(Acl,B,C,D));
```

Or manually:

```matlab
lambda = eig(Acl);

sigma = real(lambda);
omega_d = imag(lambda);
omega_n = sqrt(sigma.^2 + omega_d.^2);
zeta = -sigma ./ omega_n;
```

---

## 11. Gain Selection Criteria

A good pitch-rate feedback gain should satisfy the following:

| Criterion | Requirement |
|---|---|
| Stability | All poles remain in the left-half plane |
| Short-period damping | Damping ratio increases |
| Response speed | Settling time decreases |
| Overshoot | Overshoot decreases |
| Control effort | Elevator command remains realistic |
| Robustness | No excessive sensitivity to gain changes |

---

## 12. Recommended Gain-Selection Procedure

### Step 1

Start with open-loop gain:

```math
K_q=0
```

### Step 2

Increase the gain gradually.

### Step 3

Observe the short-period pole motion.

### Step 4

Check time-response plots for pitch rate and pitch angle.

### Step 5

Select a gain that improves damping without excessive elevator activity.

---

## 13. Expected Result

A reasonable selected gain should produce:

- Faster decay of pitch-rate oscillations.
- Lower overshoot in pitch response.
- Improved short-period damping ratio.
- Stable closed-loop poles.
- Acceptable elevator command magnitude.

For this portfolio model, a practical starting value is:

```math
K_q=1.0
```

This value should be verified using pole plots, step responses, and control-effort plots.

---

## 14. Engineering Interpretation

The root-locus plot provides a visual explanation of how feedback changes aircraft dynamics.

As `K_q` increases, the pitch-rate damper adds damping to the aircraft. This causes the short-period poles to move farther left in the complex plane.

The phugoid poles are expected to move only slightly because the pitch-rate damper mainly acts on the fast pitch dynamics.

This behavior confirms that pitch-rate feedback is an appropriate inner-loop stability augmentation method.

---

## 15. Conclusion

The root-locus design procedure was used to evaluate the effect of pitch-rate feedback gain on the longitudinal aircraft dynamics.

The selected gain should improve short-period damping, reduce oscillation, and maintain stable closed-loop behavior.

The root-locus analysis supports the final Phase 2 comparison between the open-loop and closed-loop aircraft responses.

