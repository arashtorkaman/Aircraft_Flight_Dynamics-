# Phase 3 — Autopilot and Modern Flight-Control Theory

## 1. Purpose

Phase 3 extends the longitudinal aircraft dynamics and pitch-rate damping work from the earlier portfolio phases into a command-following autopilot. The principal objective is to demonstrate the complete engineering path from a linear aircraft model to closed-loop state-feedback control, LQR design, cascaded autopilot loops, simulation, and verification.

The baseline linear time-invariant model is

$$\dot{x} = A x + B u$$

with output equation

$$y = Cx + Du$$

For a longitudinal model, a representative perturbation-state vector is

$$x = \begin{bmatrix} u & w & q & \theta \end{bmatrix}^{T}$$

where $u$ is forward-velocity perturbation, $w$ is vertical body-axis velocity perturbation, $q$ is pitch rate, and $\theta$ is pitch attitude.

The elevator is the principal longitudinal control input,

$$u_c = \delta_e$$

Depending on the model implementation, throttle may be added as a second input.

---

## 2. From Stability Augmentation to Autopilot Control

A pitch-rate damper uses a local feedback law such as

$$\delta_e = -K_q q$$

Its primary purpose is stability augmentation: increasing damping of the aircraft's longitudinal response.

An autopilot has a broader objective. It must cause the aircraft to follow a commanded state or trajectory. Examples include commanded pitch attitude, altitude, airspeed, or flight-path angle.

The conceptual progression is therefore

$$\text{Aircraft Dynamics} \rightarrow \text{Stability Augmentation} \rightarrow \text{State Feedback} \rightarrow \text{Command Tracking} \rightarrow \text{Autopilot}$$

---

## 3. Closed-Loop State-Space Dynamics

Consider full-state feedback

$$u = -Kx$$

Substitution into the plant gives

$$\dot{x} = Ax+B(-Kx)$$

and therefore

$$\boxed{\dot{x}=(A-BK)x}$$

The closed-loop state matrix is

$$\boxed{A_{cl}=A-BK}$$

The eigenvalues of $A_{cl}$ determine the local closed-loop dynamic behavior.

A successful controller should normally produce an acceptable combination of:

- stability,
- damping,
- natural frequency,
- settling time,
- overshoot,
- command tracking,
- disturbance rejection,
- actuator usage,
- robustness.

Stable poles alone do not constitute a complete controller design.

---

## 4. Reference Tracking

Pure state feedback regulates the state toward the origin. An autopilot must normally track a nonzero reference.

One possible control law is

$$u = -Kx + Nr$$

where $r$ is the reference command and $N$ is a reference-feedforward gain.

Alternatively, integral action may be introduced. Define tracking error

$$e = r-y$$

and an integral state

$$\dot{x}_I=e$$

An augmented controller can then be written as

$$u=-K_x x-K_I x_I$$

Integral action is useful when zero steady-state tracking error is required, but excessive integral gain can cause oscillation, actuator saturation, or integrator windup.

---

## 5. Optimal State Feedback

Linear Quadratic Regulator design selects the feedback gain by minimizing

$$J= \int_0^\infty \left( x^TQx+u^TRu \right)dt$$

The matrix $Q$ penalizes state deviations, while $R$ penalizes control effort.

The resulting control law is

$$u=-K_{\mathrm{LQR}}x$$

LQR does not remove the need for engineering judgment. The designer determines what matters through the selection of $Q$ and $R$ and then verifies the resulting closed-loop behavior.

---

## 6. Cascaded Autopilot Structure

Aircraft autopilots are naturally organized into nested loops because aircraft variables evolve on different time scales.

A representative longitudinal hierarchy is

$$h_c \rightarrow \theta_c \rightarrow q_c \rightarrow \delta_e$$

The inner pitch-rate loop should be faster than the attitude loop, and the attitude loop should be faster than the altitude loop.

This bandwidth separation allows the outer loop to treat the inner controlled dynamics as a comparatively fast subsystem.

A typical hierarchy is:

1. **Pitch-rate loop:** stabilizes and controls $q$.
2. **Pitch-attitude loop:** tracks $\theta_c$.
3. **Altitude loop:** converts altitude error into an attitude or flight-path command.
4. **Airspeed/throttle loop:** may be added to regulate longitudinal energy.

---

## 7. Performance Metrics

For each command-response test, evaluate at least:

### Rise time

Time required for the response to move through a specified portion of the final change.

### Settling time

Time required for the response to enter and remain inside a defined tolerance band.

### Overshoot

For a positive step,

$$M_p = \frac{y_{\max}-y_{ss}}{|y_{ss}|}\times100\%$$

### Steady-state error

$$e_{ss} = \lim_{t\rightarrow\infty} [r(t)-y(t)]$$

### Control effort

Inspect elevator deflection, elevator rate if modeled, and throttle demand if present.

---

## 8. Actuator Constraints

A mathematically stable controller can still be physically unacceptable.

The elevator should therefore be checked against constraints such as

$$|\delta_e|\leq\delta_{e,\max}$$

and, when actuator dynamics are modeled,

$$|\dot{\delta}_e|\leq\dot{\delta}_{e,\max}$$

The numerical limits must come from the selected aircraft/model assumptions and should be documented rather than invented.

---

## 9. Verification Philosophy

Phase 3 should verify the controller at several levels:

1. Confirm controllability of the selected state-space model.
2. Compare open-loop and closed-loop poles.
3. Test state and output responses.
4. Verify reference tracking.
5. Check actuator demand.
6. Perturb initial conditions.
7. Introduce disturbances if modeled.
8. Vary model parameters to examine sensitivity.
9. Compare MATLAB and Simulink implementations.
10. Record limitations and assumptions.

The final engineering claim should be proportional to the evidence. A linear simulation demonstrates local closed-loop behavior around the chosen operating point; it does not by itself establish full-envelope aircraft performance.

---

## 10. Phase 3 Connection to Phase 4

Phase 3 establishes the control-design methodology using a linear aircraft model.

Phase 4 extends the plant to nonlinear six-degree-of-freedom dynamics,

$$\dot{x}=f(x,u,t)$$

followed by trim and local linearization,

$$\delta\dot{x}=A\delta x+B\delta u$$

This creates a direct engineering bridge between the linear autopilot and the later nonlinear aircraft simulation.
