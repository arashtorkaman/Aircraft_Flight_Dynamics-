# Controller Tuning and Verification

## 1. Purpose

Controller tuning converts a mathematically stable design into a controller that provides acceptable aircraft response while respecting control effort, bandwidth, and model limitations.

The objective is not merely to eliminate oscillation. Aircraft responses may legitimately contain damped oscillatory modes. The engineering requirement is that the response exhibit acceptable damping, convergence, tracking, and actuator demand.

---

## 2. Establish the Baseline

Before tuning, record:

- trim/operating condition,
- state definition and units,
- $A$, $B$, $C$, $D$ matrices,
- open-loop poles,
- damping ratios,
- natural frequencies,
- control inputs,
- actuator limits used by the model.

This baseline prevents tuning decisions from becoming disconnected from the actual aircraft model.

---

## 3. Tune from the Inside Out

For a cascaded autopilot, tune in this order:

1. pitch-rate/stability loop,
2. pitch-attitude loop,
3. altitude or flight-path loop,
4. airspeed/throttle loop,
5. combined autopilot.

Do not tune the outer altitude loop while the inner attitude dynamics are still unacceptable.

---

## 4. Pitch-Rate Loop

The pitch-rate loop should improve damping without producing excessive elevator activity.

For

$$\delta_e=-K_qq$$

increase $K_q$ gradually and monitor:

- short-period poles,
- pitch-rate decay,
- pitch attitude,
- elevator command,
- sensitivity to noise.

Too little feedback may provide insufficient damping.

Too much feedback can create excessive actuator demand, amplify measurement noise, or interact poorly with neglected actuator dynamics.

---

## 5. Attitude Loop

With the inner loop closed, tune the attitude loop.

For a simple proportional relationship,

$$q_c=K_\theta(\theta_c-\theta)$$

Increase $K_\theta$ until the attitude response is sufficiently fast, but retain adequate damping and separation from the inner-loop bandwidth.

Record:

- rise time,
- settling time,
- overshoot,
- peak $q$,
- peak elevator.

---

## 6. Altitude Loop

Once attitude control is verified,

$$e_h=h_c-h$$

can be converted into an attitude or flight-path command.

The altitude loop should be deliberately slower than the attitude loop.

Large altitude steps can generate excessive attitude commands, so include command limits and use test commands appropriate to the validity range of the linear model.

---

## 7. LQR Tuning

For

$$J= \int_0^\infty (x^TQx+u^TRu)dt$$

use a repeatable tuning process.

### Step 1 — Scale the states

Identify meaningful maximum excursions for each state.

### Step 2 — Select baseline weights

A useful starting point is

$$Q_{ii}\sim\frac{1}{x_{i,\max}^2}, \qquad R_{jj}\sim\frac{1}{u_{j,\max}^2}$$

### Step 3 — Run the baseline

Store all metrics and plots.

### Step 4 — Change one design emphasis at a time

Examples:

- increase $q$ weight,
- increase $\theta$ weight,
- increase velocity weight,
- increase $R$.

### Step 5 — Compare objectively

Do not choose a controller only because one plot appears smoother.

---

## 8. Performance Table

Use a table such as:

| Design | Rise time | Settling time | Overshoot | Steady-state error | Peak elevator | Notes |
|---|---:|---:|---:|---:|---:|---|
| Baseline | TBD | TBD | TBD | TBD | TBD | |
| Tuning A | TBD | TBD | TBD | TBD | TBD | |
| Tuning B | TBD | TBD | TBD | TBD | TBD | |
| Final | TBD | TBD | TBD | TBD | TBD | |

Populate values directly from simulation results.

---

## 9. Steady-State Behavior

For a stable LTI system under a constant input,

$$\dot{x}=Ax+Bu$$

the steady-state state vector satisfies

$$0=Ax_{ss}+Bu$$

Therefore, if $A$ is nonsingular,

$$\boxed{x_{ss}=-A^{-1}Bu}$$

In MATLAB:

```matlab
x_ss = -A \ (B*u_cmd);
```

For the closed-loop system, use the actual closed-loop input architecture. If the model is

$$\dot{x}=A_{cl}x+B_rr$$

then

$$x_{ss}=-A_{cl}^{-1}B_rr$$

A stable aircraft response does **not** have to be monotonic. A stable complex pole pair produces damped oscillation that converges toward equilibrium.

---

## 10. Saturation and Rate Limits

Add saturation to the implemented controller using the documented actuator range.

If actuator rate is modeled,

$$|\dot{\delta}_e| \leq \dot{\delta}_{e,\max}$$

A controller that repeatedly hits saturation should not be described as acceptable solely because the unsaturated linear model is stable.

If integral action is present, include anti-windup or document it as a required improvement.

---

## 11. Disturbance Tests

After nominal command tracking works, test disturbances appropriate to the model.

Examples include:

- initial pitch-rate disturbance,
- pitch-attitude offset,
- vertical gust approximation,
- velocity perturbation.

For each test, examine whether the aircraft returns to the commanded condition without excessive oscillation or control demand.

---

## 12. Parameter Sensitivity

Vary selected aircraft parameters or derivatives and rerun the same tests.

A simple study can evaluate nominal, reduced, and increased values for a small set of influential parameters.

The goal is to answer:

> Does the controller remain stable and reasonably behaved when the model is not exactly nominal?

This is a robustness screening exercise, not a certification claim.

---

## 13. Numerical and Simulation Checks

Use:

```matlab
eig(Acl)
damp(sys_cl)
```

and inspect:

- time vector resolution,
- solver settings,
- simulation duration,
- initial conditions,
- command magnitude,
- units,
- sign conventions.

A long-period phugoid mode may require a substantially longer simulation than the short-period mode before convergence is visually obvious.

---

## 14. Controller Selection Criteria

The final controller should be selected using explicit criteria:

- all required closed-loop modes stable,
- acceptable damping,
- acceptable tracking,
- acceptable steady-state error,
- reasonable settling time,
- limited overshoot,
- actuator demand within assumed limits,
- acceptable sensitivity to perturbations,
- consistent MATLAB/Simulink behavior.

If a requirement has not been specified numerically, report the observed metric rather than claiming that it meets an undefined requirement.

---

## 15. Documentation Rule

Every final gain should be traceable to:

1. the plant version,
2. the operating point,
3. the design method,
4. the selected weights/poles,
5. the simulation cases,
6. the performance results.

This makes Phase 3 reproducible and prepares the project for later model governance, requirements traceability, and certification-oriented workflows.
