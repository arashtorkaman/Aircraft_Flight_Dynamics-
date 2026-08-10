# Linear Quadratic Regulator Design

## 1. Purpose

The Linear Quadratic Regulator provides a systematic method for designing full-state feedback for the linear aircraft model.

Given

$$\dot{x}=Ax+Bu$$

LQR determines a gain matrix $K$ for

$$u=-Kx$$

by minimizing a quadratic performance index.

---

## 2. Cost Function

The continuous-time infinite-horizon cost is

$$\boxed{ J= \int_0^\infty \left( x^TQx+u^TRu \right)dt }$$

$Q$ is the state-weighting matrix and $R$ is the control-weighting matrix.

For

$$x= \begin{bmatrix} u&w&q&\theta \end{bmatrix}^{T}$$

a diagonal starting point may be written

$$Q= \operatorname{diag} \left( q_u,q_w,q_q,q_\theta \right)$$

For a single elevator input,

$$R= \begin{bmatrix} r_{\delta_e} \end{bmatrix}$$

The values are design parameters and must be tuned for the actual aircraft model.

---

## 3. Interpretation of the Weights

Increasing a state weight tells the optimizer that deviation in that state is more costly.

For example, increasing $q_\theta$ generally places greater emphasis on pitch-attitude regulation.

Increasing $q_q$ generally places greater emphasis on pitch-rate regulation.

Increasing $R$ penalizes elevator activity more heavily and generally produces a less aggressive controller.

Thus the design problem is not simply "find the largest $Q$." It is a trade between state performance and physically reasonable control effort.

---

## 4. Riccati Equation

The optimal solution is obtained from the continuous-time algebraic Riccati equation

$$A^TP+PA-PBR^{-1}B^TP+Q=0$$

The state-feedback gain is

$$\boxed{ K=R^{-1}B^TP }$$

MATLAB performs this calculation directly:

```matlab
[K,S,e] = lqr(A,B,Q,R);
```

where `K` is the optimal feedback gain, `S` is the Riccati solution, and `e` contains the closed-loop eigenvalues.

---

## 5. State Scaling

LQR weights should be interpreted in relation to the physical scale of each state.

A useful starting methodology is Bryson-style scaling:

$$Q_{ii}\approx\frac{1}{x_{i,\max}^2}$$

and

$$R_{jj}\approx\frac{1}{u_{j,\max}^2}$$

where the maxima represent meaningful acceptable excursions rather than arbitrary numbers.

This helps prevent units and state magnitudes from dominating the optimization unintentionally.

The resulting controller still requires simulation and engineering tuning.

---

## 6. Basic MATLAB Design

```matlab
% Example structure only: replace weights with justified values.
Q = diag([q_u q_w q_q q_theta]);
R = r_elevator;

[K,S,e] = lqr(A,B,Q,R);

Acl = A - B*K;

disp("LQR gain:")
disp(K)

disp("Closed-loop poles:")
disp(e)
```

---

## 7. LQR Tuning Study

Do not publish only one arbitrary $Q,R$ pair.

Perform a controlled tuning study.

### Case A — Baseline

Select balanced initial weights.

### Case B — Stronger attitude weighting

Increase the pitch-attitude weight and observe:

- pitch tracking,
- settling time,
- elevator demand,
- changes in other states.

### Case C — Stronger pitch-rate weighting

Increase the $q$ weight and observe changes in damping and control activity.

### Case D — Increased control penalty

Increase $R$ and observe the performance/control-effort trade.

Create a table such as:

| Case | Key weighting change | Settling time | Overshoot | Peak elevator | Assessment |
|---|---|---:|---:|---:|---|
| A | Baseline | TBD | TBD | TBD | TBD |
| B | Higher attitude weight | TBD | TBD | TBD | TBD |
| C | Higher pitch-rate weight | TBD | TBD | TBD | TBD |
| D | Higher control penalty | TBD | TBD | TBD | TBD |

Do not populate the table until the simulations have been run.

---

## 8. Reference Tracking with LQR

Standard LQR is a regulator.

For command tracking, combine LQR with a reference prefilter or integral augmentation.

One possible law is

$$u=-Kx+Nr$$

For integral tracking,

$$\dot{x}_I=r-y$$

and

$$u=-K_x x-K_I x_I$$

An augmented LQR design may be constructed by adding the integral state to the plant before solving the LQR problem.

---

## 9. Actuator Saturation

The optimal solution is optimal for the mathematical model and cost function used in the design. It does not automatically enforce actuator limits.

After calculating

$$u(t)=-Kx(t)$$

verify that the elevator command remains within the documented actuator limits.

If saturation is present, the linear closed-loop response predicted by $A-BK$ no longer fully represents the implemented system.

---

## 10. Robustness Checks

At minimum, test sensitivity to plausible perturbations in the model used for the portfolio, such as:

- aerodynamic derivative changes,
- mass variation,
- inertia variation,
- initial-condition errors,
- sensor noise if modeled,
- disturbances if modeled.

The purpose is not to claim certification-level robustness. It is to show awareness that an LQR design based on one exact matrix pair $(A,B)$ must be tested against uncertainty.

---

## 11. LQR versus Pole Placement

Pole placement asks:

> Where should the closed-loop poles be?

LQR asks:

> What state/control trade produces the minimum quadratic cost?

Both produce a feedback matrix $K$, but they encode the design objective differently.

A strong Phase 3 portfolio can compare them using the same plant and the same command or initial-condition tests.

---

## 12. Required Figures

Recommended figures include:

1. open-loop versus LQR pole map,
2. pitch attitude response,
3. pitch-rate response,
4. altitude response if altitude dynamics are included,
5. airspeed response,
6. elevator command,
7. comparison of multiple $Q/R$ tuning cases.

Each figure should include units, legend where needed, descriptive title, and the command/reference when applicable.
