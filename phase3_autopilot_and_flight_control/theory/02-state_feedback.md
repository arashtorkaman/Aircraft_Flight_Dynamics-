# State-Feedback Control Design

## 1. Objective

State feedback uses measured or estimated aircraft states to modify the control input so that the closed-loop aircraft exhibits desired dynamics.

Start with

$$\dot{x}=Ax+Bu$$

For the longitudinal aircraft model, a representative state vector is

$$x= \begin{bmatrix} u&w&q&\theta \end{bmatrix}^{T}$$

The exact state ordering must match the MATLAB model used in the repository.

---

## 2. Feedback Law

For full-state feedback,

$$\boxed{u=-Kx}$$

If

$$K= \begin{bmatrix} k_u&k_w&k_q&k_\theta \end{bmatrix}$$

then the elevator command is

$$\delta_e = -k_u u -k_w w -k_q q -k_\theta\theta$$

The pitch-rate damper from Phase 2 is therefore a special case of state feedback in which only the $q$ state is fed back.

---

## 3. Closed-Loop Model

Substituting the control law into the aircraft model,

$$\dot{x} = Ax-BKx$$

so

$$\boxed{\dot{x}=A_{cl}x}$$

where

$$\boxed{A_{cl}=A-BK}$$

Closed-loop poles are

$$\lambda_i=\operatorname{eig}(A-BK)$$

For a continuous-time linear system, asymptotic stability requires

$$\Re(\lambda_i)<0$$

for every closed-loop eigenvalue.

---

## 4. Controllability

Before attempting arbitrary pole placement, evaluate the controllability matrix

$$\mathcal{C} = \begin{bmatrix} B&AB&A^2B&\cdots&A^{n-1}B \end{bmatrix}$$

The system is completely controllable when

$$\operatorname{rank}(\mathcal{C})=n$$

In MATLAB:

```matlab
Co = ctrb(A,B);
rank_Co = rank(Co);
n = size(A,1);

if rank_Co == n
    disp("System is controllable")
else
    warning("System is not fully controllable")
end
```

This check should appear in the design workflow before pole placement.

---

## 5. Pole Placement

When the system is controllable, desired poles can be selected and the state-feedback gain computed.

```matlab
desired_poles = [...];
K = place(A,B,desired_poles);
Acl = A - B*K;
eig(Acl)
```

The desired pole locations should be justified from dynamic requirements rather than selected only because they produce a visually smooth plot.

Important considerations include:

- desired damping ratio,
- desired natural frequency,
- separation of fast and slow modes,
- actuator demand,
- model uncertainty,
- noise sensitivity.

---

## 6. Pole Parameters

For a complex pole pair

$$s=-\sigma\pm j\omega_d$$

the natural frequency is

$$\omega_n=\sqrt{\sigma^2+\omega_d^2}$$

and damping ratio is

$$\zeta=\frac{\sigma}{\omega_n}$$

These quantities provide a more useful interpretation than pole coordinates alone.

---

## 7. Command Tracking

The regulator law

$$u=-Kx$$

drives the states toward zero.

For nonzero reference tracking, use an appropriate reference architecture. A simple form is

$$u=-Kx+Nr$$

For a SISO system with compatible definitions, $N$ can be selected so the steady-state output tracks a constant reference. The exact expression depends on the plant and selected output.

Another approach is integral augmentation.

Define

$$\dot{x}_I=r-y$$

Then

$$u=-K_x x-K_I x_I$$

The augmented state becomes

$$x_a= \begin{bmatrix} x\\x_I \end{bmatrix}$$

---

## 8. State Measurement and Estimation

Full-state feedback assumes the states are available to the controller.

In a real aircraft, not every state is measured directly. Measurements may come from:

- inertial sensors,
- air-data sensors,
- attitude and heading systems,
- navigation sensors.

Unmeasured states may require an observer or Kalman filter.

For Phase 3, using the simulated states directly is acceptable provided this assumption is documented. State estimation can be introduced as a later portfolio extension.

---

## 9. Comparison with Pitch-Rate Feedback

Phase 2:

$$\delta_e=-K_qq$$

Phase 3 full-state feedback:

$$\delta_e=-Kx$$

The first primarily modifies a selected mode through one feedback variable. The second can coordinate several states simultaneously and therefore provides a foundation for multivariable autopilot design.

---

## 10. MATLAB Verification

A basic verification sequence is:

```matlab
% Open-loop poles
p_ol = eig(A);

% Controllability
Co = ctrb(A,B);
rank_Co = rank(Co);

% State-feedback gain
K = place(A,B,desired_poles);

% Closed-loop matrix
Acl = A - B*K;

% Closed-loop poles
p_cl = eig(Acl);

% Closed-loop system
sys_cl = ss(Acl,B,C,D);
```

For a regulator initial-condition test:

```matlab
initial(sys_cl, x0, t);
grid on
```

Reference tracking should be simulated using the actual reference-input architecture rather than treating the regulator input incorrectly as a command.

---

## 11. Required Results

Document:

- open-loop poles,
- controllability rank,
- selected desired poles,
- resulting gain matrix $K$,
- closed-loop poles,
- damping ratios,
- natural frequencies,
- state histories,
- output tracking,
- elevator command,
- saturation status.

The report should explain *why* the selected controller is acceptable rather than simply displaying plots.
