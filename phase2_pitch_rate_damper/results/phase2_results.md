# Phase 2 Results - Pitch Rate Damper

## Longitudinal Stability Augmentation Results

---

## 1. Objective

The objective of Phase 2 was to design and evaluate a pitch-rate damper for the longitudinal aircraft model.

The pitch-rate damper uses pitch-rate feedback to command elevator deflection and improve short-period damping.

The control law is:

```math
\delta_e=-K_q q
```

The closed-loop model is:

```math
A_{cl}=A-BK
```

---

## 2. Open-Loop Model Summary

The open-loop longitudinal model is:

```math
\dot{x}=Ax+B\delta_e
```

The state vector is:

```math
x=[u,\;w,\;q,\;\theta]^T
```

The open-loop system contains two important longitudinal modes:

| Mode | Description |
|---|---|
| Short-period | Fast pitch and angle-of-attack motion |
| Phugoid | Slow exchange between kinetic and potential energy |

Typical open-loop poles are:

```math
\lambda_{SP}=-3.2548\pm1.7168j
```

```math
\lambda_{PH}=-0.0052\pm0.1461j
```

---

## 3. Closed-Loop Controller

The selected controller is a pitch-rate feedback controller:

```math
\delta_e=-K_q q
```

The feedback vector is:

```math
K=[0\;\;0\;\;K_q\;\;0]
```

The selected nominal gain for the baseline closed-loop comparison is:

```math
K_q=1.0
```

This value is used as an initial practical gain and should be refined using root-locus and time-domain analysis.

---

## 4. Closed-Loop Model

The closed-loop system matrix is:

```math
A_{cl}=A-BK
```

The closed-loop dynamics are:

```math
\dot{x}=A_{cl}x
```

For simulation with an external elevator command input:

```math
\dot{x}=A_{cl}x+B\delta_{cmd}
```

---

## 5. Pole Comparison

The main purpose of the pole comparison is to confirm that the pitch-rate damper improves short-period damping.

Expected pole behavior:

| Mode | Expected Closed-Loop Change |
|---|---|
| Short-period | Poles move farther left |
| Phugoid | Small change only |

A successful pitch-rate damper should keep all poles in the left-half plane.

---

## 6. Expected Dynamic Improvements

The closed-loop system should show:

- Increased short-period damping.
- Reduced pitch-rate oscillation.
- Reduced pitch-attitude overshoot.
- Faster settling time.
- Smoother aircraft response.

The phugoid mode may remain lightly damped because pitch-rate feedback mainly targets the short-period mode.

---

## 7. Open-Loop vs Closed-Loop Response

The open-loop and closed-loop systems should be compared using the same elevator input.

Recommended input:

```math
\delta_e=1^\circ
```

In MATLAB:

```matlab
input_deg = 1;
input_rad = deg2rad(input_deg);
```

The closed-loop response should converge faster and with less oscillation.

---

## 8. MATLAB Result Script

```matlab
% Define pitch-rate feedback gain
Kq = 1.0;
K = [0 0 Kq 0];

% Closed-loop model
Acl = A - B*K;

% State-space systems
sys_ol = ss(A,B,C,D);
sys_cl = ss(Acl,B,C,D);

% Poles
poles_ol = eig(A);
poles_cl = eig(Acl);

disp('Open-loop poles:');
disp(poles_ol);

disp('Closed-loop poles:');
disp(poles_cl);
```

---

## 9. Step Response Plot

```matlab
figure;
step(sys_ol);
hold on;
step(sys_cl);
grid on;
legend('Open Loop','Closed Loop');
title('Open-Loop vs Closed-Loop Step Response');
```

For clearer engineering interpretation, plot individual state responses:

```matlab
t = 0:0.01:20;
u_step = deg2rad(1)*ones(size(t));

[y_ol,t_ol,x_ol] = lsim(sys_ol,u_step,t);
[y_cl,t_cl,x_cl] = lsim(sys_cl,u_step,t);
```

---

## 10. Pitch-Rate Response

The pitch-rate response is one of the most important Phase 2 outputs.

Expected result:

| Quantity | Open Loop | Closed Loop |
|---|---|---|
| Pitch-rate oscillation | Larger | Smaller |
| Decay rate | Slower | Faster |
| Settling time | Longer | Shorter |

MATLAB plot:

```matlab
figure;
plot(t_ol,x_ol(:,3));
hold on;
plot(t_cl,x_cl(:,3));
grid on;
xlabel('Time [s]');
ylabel('Pitch Rate q [rad/s]');
legend('Open Loop','Closed Loop');
title('Pitch Rate Response');
```

---

## 11. Pitch-Attitude Response

The pitch-attitude response shows how the aircraft attitude changes after elevator input.

Expected result:

- Reduced overshoot.
- Smoother transient response.
- Faster convergence toward steady state.

MATLAB plot:

```matlab
figure;
plot(t_ol,rad2deg(x_ol(:,4)));
hold on;
plot(t_cl,rad2deg(x_cl(:,4)));
grid on;
xlabel('Time [s]');
ylabel('Pitch Attitude theta [deg]');
legend('Open Loop','Closed Loop');
title('Pitch Attitude Response');
```

---

## 12. Elevator Command Response

The closed-loop elevator command is:

```math
\delta_e=-K_q q
```

MATLAB calculation:

```matlab
delta_e_cl = -Kq*x_cl(:,3);

figure;
plot(t_cl,rad2deg(delta_e_cl));
grid on;
xlabel('Time [s]');
ylabel('Elevator Command [deg]');
title('Closed-Loop Elevator Command');
```

The elevator command should remain within realistic actuator limits.

---

## 13. Result Figures to Include

Place final plots in the `figures` folder.

Recommended files:

| Figure | Filename |
|---|---|
| Open-loop pole map | `open_loop_poles.png` |
| Closed-loop pole map | `closed_loop_poles.png` |
| Root-locus plot | `root_locus.png` |
| Step response comparison | `step_response_comparison.png` |
| Pitch-rate response | `pitch_rate_response.png` |
| Pitch-attitude response | `pitch_attitude_response.png` |
| Elevator command | `elevator_command.png` |

---

## 14. Results Table Template

Use this table after running the MATLAB simulation.

| Metric | Open Loop | Closed Loop | Improvement |
|---|---:|---:|---|
| Short-period damping ratio | TBD | TBD | Increased |
| Short-period natural frequency | TBD | TBD | TBD |
| Pitch-rate settling time | TBD | TBD | Reduced |
| Pitch-attitude overshoot | TBD | TBD | Reduced |
| Maximum elevator command | N/A | TBD | Check actuator limit |

---

## 15. Engineering Interpretation

The pitch-rate damper improves the short-period response by feeding back pitch rate to the elevator.

The closed-loop system behaves as if additional pitch damping has been added to the aircraft.

This reduces rapid pitch oscillations and improves handling qualities.

The expected engineering conclusion is that pitch-rate feedback is effective as an inner-loop stability augmentation system.

---

## 16. Conclusion

Phase 2 successfully introduced a closed-loop pitch-rate damper into the longitudinal aircraft model.

The controller:

```math
\delta_e=-K_q q
```

modifies the aircraft dynamics through:

```math
A_{cl}=A-BK
```

The expected closed-loop response shows improved damping, reduced oscillation, and faster settling compared with the open-loop aircraft.

This completes the first control-system design step in the longitudinal flight dynamics portfolio and prepares the project for more advanced autopilot development.
