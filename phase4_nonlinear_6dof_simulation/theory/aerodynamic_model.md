# Aerodynamic Model

## Project Context

This document defines the aerodynamic force and moment model used for the **F-22-inspired MIMO 6-DOF fighter aircraft simulation**.

The model is intended for educational flight-dynamics, control-design, and portfolio demonstration purposes. It is **not** a representation of the actual F-22 aerodynamic database, proprietary stability derivatives, flight-control laws, or classified aircraft data.

The purpose of this document is to define a clean coefficient-based aerodynamic model that can be implemented in MATLAB and connected to the nonlinear 6-DOF equations of motion.

---

## 1. Role of the Aerodynamic Model

The nonlinear 6-DOF aircraft model requires total body-axis forces and moments:

$$
\mathbf{F}_b =
\begin{bmatrix}
X \\
Y \\
Z
\end{bmatrix}
$$

$$
\mathbf{M}_b =
\begin{bmatrix}
L \\
M \\
N
\end{bmatrix}
$$

where:

| Symbol | Meaning |
|---|---|
| $X$ | Body-axis force along the forward body axis |
| $Y$ | Body-axis side force |
| $Z$ | Body-axis vertical force, positive downward |
| $L$ | Rolling moment about the body $x$-axis |
| $M$ | Pitching moment about the body $y$-axis |
| $N$ | Yawing moment about the body $z$-axis |

The aerodynamic model provides these quantities using nondimensional aerodynamic coefficients:

$$
C_X, \quad C_Y, \quad C_Z, \quad C_l, \quad C_m, \quad C_n
$$

These coefficients are functions of aircraft state, angular rates, and control inputs.

---

## 2. State and Control Inputs Used by the Aerodynamic Model

The aerodynamic model uses the aircraft body-axis velocity states:

$$
u, \quad v, \quad w
$$

body-axis angular rates:

$$
p, \quad q, \quad r
$$

and control inputs:

$$
\delta_e, \quad \delta_a, \quad \delta_r, \quad \delta_T, \quad \delta_{tv}
$$

where:

| Input | Meaning |
|---|---|
| $\delta_e$ | Elevator or stabilator deflection |
| $\delta_a$ | Aileron deflection |
| $\delta_r$ | Rudder deflection |
| $\delta_T$ | Normalized throttle command |
| $\delta_{tv}$ | Pitch-axis thrust-vectoring nozzle command |

For this educational model, $\delta_T$ is nondimensional and constrained between 0 and 1:

$$
0 \leq \delta_T \leq 1
$$

The control-surface deflections are expressed in radians.

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

A numerically safer implementation uses the two-argument arctangent:

$$
\alpha = \operatorname{atan2}(w,u)
$$

The sideslip angle is:

$$
\beta = \sin^{-1}\left(\frac{v}{V}\right)
$$

In MATLAB, the argument of the inverse sine should be limited to the interval $[-1,1]$ to avoid numerical errors:

```matlab
beta = asin(max(min(v/V,1),-1));
```

---

## 4. Dynamic Pressure

Dynamic pressure is defined as:

$$
\bar{q} = \frac{1}{2}\rho V^2
$$

where:

| Symbol | Meaning |
|---|---|
| $\bar{q}$ | Dynamic pressure |
| $\rho$ | Air density |
| $V$ | Total airspeed |

The dynamic pressure scales aerodynamic forces and moments. At higher airspeed, the same aerodynamic coefficient produces a larger force or moment.

---

## 5. Nondimensional Angular Rates

The nondimensional roll, pitch, and yaw rates are:

$$
\hat{p} = \frac{pb}{2V}
$$

$$
\hat{q} = \frac{q\bar{c}}{2V}
$$

$$
\hat{r} = \frac{rb}{2V}
$$

where:

| Symbol | Meaning |
|---|---|
| $b$ | Wingspan |
| $\bar{c}$ | Mean aerodynamic chord |
| $V$ | Airspeed |

These nondimensional rates allow damping derivatives such as $C_{m_{\hat{q}}}$, $C_{l_{\hat{p}}}$, and $C_{n_{\hat{r}}}$ to be used consistently.

---

## 6. Lift, Drag, and Side-Force Coefficients

For the first version of the model, the aerodynamic lift coefficient is modeled as:

$$
C_L = C_{L0} + C_{L_{\alpha}}\alpha + C_{L_{\hat{q}}}\hat{q} + C_{L_{\delta_e}}\delta_e
$$

The drag coefficient is modeled using a parabolic drag polar:

$$
C_D = C_{D0} + K C_L^2
$$

The side-force coefficient is modeled as:

$$
C_Y = C_{Y_{\beta}}\beta
+ C_{Y_{\hat{p}}}\hat{p}
+ C_{Y_{\hat{r}}}\hat{r}
+ C_{Y_{\delta_a}}\delta_a
+ C_{Y_{\delta_r}}\delta_r
$$

where:

| Coefficient | Meaning |
|---|---|
| $C_{L0}$ | Lift coefficient at zero angle of attack |
| $C_{L_{\alpha}}$ | Lift-curve slope |
| $C_{L_{\hat{q}}}$ | Pitch-rate lift contribution |
| $C_{L_{\delta_e}}$ | Elevator lift effectiveness |
| $C_{D0}$ | Zero-lift drag coefficient |
| $K$ | Induced-drag factor |
| $C_{Y_{\beta}}$ | Side-force derivative with sideslip |
| $C_{Y_{\hat{p}}}$ | Side-force derivative with roll rate |
| $C_{Y_{\hat{r}}}$ | Side-force derivative with yaw rate |
| $C_{Y_{\delta_a}}$ | Side-force derivative with aileron deflection |
| $C_{Y_{\delta_r}}$ | Side-force derivative with rudder deflection |

---

## 7. Moment Coefficients

The rolling-moment coefficient is modeled as:

$$
C_l = C_{l_{\beta}}\beta
+ C_{l_{\hat{p}}}\hat{p}
+ C_{l_{\hat{r}}}\hat{r}
+ C_{l_{\delta_a}}\delta_a
+ C_{l_{\delta_r}}\delta_r
$$

The pitching-moment coefficient is modeled as:

$$
C_m = C_{m0}
+ C_{m_{\alpha}}\alpha
+ C_{m_{\hat{q}}}\hat{q}
+ C_{m_{\delta_e}}\delta_e
+ C_{m_{\delta_{tv}}}\delta_{tv}
$$

The yawing-moment coefficient is modeled as:

$$
C_n = C_{n_{\beta}}\beta
+ C_{n_{\hat{p}}}\hat{p}
+ C_{n_{\hat{r}}}\hat{r}
+ C_{n_{\delta_a}}\delta_a
+ C_{n_{\delta_r}}\delta_r
$$

The signs of the main stability derivatives should be selected to produce physically reasonable aircraft behavior:

| Stability derivative | Required sign | Physical meaning |
|---|---:|---|
| $C_{m_{\alpha}}$ | $< 0$ | Longitudinal static stability in the simplified model |
| $C_{m_{\hat{q}}}$ | $< 0$ | Pitch-rate damping |
| $C_{l_{\hat{p}}}$ | $< 0$ | Roll-rate damping |
| $C_{n_{\hat{r}}}$ | $< 0$ | Yaw-rate damping |
| $C_{n_{\beta}}$ | $> 0$ | Directional weathercock stability |

For a real highly maneuverable fighter, the aircraft may be statically relaxed or unstable in some regimes. This simplified model begins with a stable baseline so that trim, linearization, and MIMO LQR control can be developed progressively.

---

## 8. Conversion from Lift and Drag to Body-Axis Force Coefficients

The 6-DOF equations require body-axis force coefficients $C_X$, $C_Y$, and $C_Z$.

Using angle of attack $\alpha$, lift and drag can be converted approximately into body-axis coefficients as:

$$
C_X = -C_D\cos\alpha + C_L\sin\alpha
$$

$$
C_Z = -C_D\sin\alpha - C_L\cos\alpha
$$

The side-force coefficient is already a body-axis lateral force coefficient:

$$
C_Y = C_Y
$$

The body-axis force coefficients are then:

$$
\mathbf{C}_F =
\begin{bmatrix}
C_X \\
C_Y \\
C_Z
\end{bmatrix}
$$

---

## 9. Aerodynamic Forces and Moments

The aerodynamic body-axis forces are:

$$
X_{aero} = \bar{q}S C_X
$$

$$
Y_{aero} = \bar{q}S C_Y
$$

$$
Z_{aero} = \bar{q}S C_Z
$$

The aerodynamic body-axis moments are:

$$
L_{aero} = \bar{q}Sb C_l
$$

$$
M_{aero} = \bar{q}S\bar{c} C_m
$$

$$
N_{aero} = \bar{q}Sb C_n
$$

where:

| Symbol | Meaning |
|---|---|
| $S$ | Reference wing area |
| $b$ | Wingspan |
| $\bar{c}$ | Mean aerodynamic chord |
| $\bar{q}$ | Dynamic pressure |

---

## 10. Propulsion and Thrust-Vectoring Model

For the first project version, total thrust is modeled as:

$$
T = \delta_T T_{max}
$$

where:

| Symbol | Meaning |
|---|---|
| $T$ | Total engine thrust |
| $\delta_T$ | Normalized throttle command |
| $T_{max}$ | Maximum total thrust |

A simplified pitch-axis thrust-vectoring model can be included by resolving thrust into body-axis components:

$$
X_T = T\cos\delta_{tv}
$$

$$
Z_T = -T\sin\delta_{tv}
$$

For small thrust-vectoring angles:

$$
X_T \approx T
$$

$$
Z_T \approx -T\delta_{tv}
$$

If the thrust-vectoring nozzle produces an additional pitching moment through an effective moment arm $l_T$, then:

$$
M_T = -l_T T\sin\delta_{tv}
$$

For small angles:

$$
M_T \approx -l_T T\delta_{tv}
$$

The total forces and moments used in the nonlinear 6-DOF equations are:

$$
X = X_{aero} + X_T
$$

$$
Y = Y_{aero}
$$

$$
Z = Z_{aero} + Z_T
$$

$$
L = L_{aero}
$$

$$
M = M_{aero} + M_T
$$

$$
N = N_{aero}
$$

---

## 11. Example Baseline Coefficient Values

The following values are representative educational starting points. They are not real F-22 aerodynamic derivatives.

### 11.1 Longitudinal Coefficients

| Parameter | Example value | Comment |
|---|---:|---|
| $C_{L0}$ | $0.20$ | Baseline lift coefficient |
| $C_{L_{\alpha}}$ | $4.80$ | Lift-curve slope |
| $C_{L_{\hat{q}}}$ | $7.50$ | Pitch-rate lift contribution |
| $C_{L_{\delta_e}}$ | $0.35$ | Elevator lift effectiveness |
| $C_{D0}$ | $0.025$ | Zero-lift drag coefficient |
| $K$ | $0.09$ | Induced-drag factor |
| $C_{m0}$ | $0.02$ | Baseline pitching moment |
| $C_{m_{\alpha}}$ | $-0.80$ | Static pitch stability derivative |
| $C_{m_{\hat{q}}}$ | $-12.0$ | Pitch-rate damping derivative |
| $C_{m_{\delta_e}}$ | $-1.10$ | Elevator pitching-moment effectiveness |
| $C_{m_{\delta_{tv}}}$ | $-0.80$ | Pitch thrust-vectoring effectiveness |

### 11.2 Lateral-Directional Coefficients

| Parameter | Example value | Comment |
|---|---:|---|
| $C_{Y_{\beta}}$ | $-0.90$ | Side-force due to sideslip |
| $C_{Y_{\hat{p}}}$ | $0.00$ | Side-force due to roll rate |
| $C_{Y_{\hat{r}}}$ | $0.25$ | Side-force due to yaw rate |
| $C_{Y_{\delta_a}}$ | $0.00$ | Side-force due to aileron |
| $C_{Y_{\delta_r}}$ | $0.18$ | Side-force due to rudder |
| $C_{l_{\beta}}$ | $-0.12$ | Dihedral-effect derivative |
| $C_{l_{\hat{p}}}$ | $-0.55$ | Roll-rate damping derivative |
| $C_{l_{\hat{r}}}$ | $0.18$ | Roll-yaw coupling derivative |
| $C_{l_{\delta_a}}$ | $0.22$ | Aileron roll effectiveness |
| $C_{l_{\delta_r}}$ | $0.02$ | Rudder roll coupling |
| $C_{n_{\beta}}$ | $0.18$ | Directional stability derivative |
| $C_{n_{\hat{p}}}$ | $-0.04$ | Yaw moment due to roll rate |
| $C_{n_{\hat{r}}}$ | $-0.30$ | Yaw-rate damping derivative |
| $C_{n_{\delta_a}}$ | $0.01$ | Aileron yaw coupling |
| $C_{n_{\delta_r}}$ | $-0.12$ | Rudder yaw effectiveness |

These values should be tuned carefully to obtain reasonable open-loop and closed-loop responses. They are intended as a starting point for simulation, not as validated aerodynamic data.

---

## 12. MATLAB Implementation Structure

A recommended MATLAB function interface is:

```matlab
function aero = aero_coefficients_fighter(x,u_ctrl,params)
```

where:

```matlab
x = [u; v; w; p; q; r; phi; theta; psi; XE; YE; ZE];
u_ctrl = [de; da; dr; dT; dtv];
```

The function should return:

```matlab
aero.CX = CX;
aero.CY = CY;
aero.CZ = CZ;
aero.Cl = Cl;
aero.Cm = Cm;
aero.Cn = Cn;

aero.X = X_total;
aero.Y = Y_total;
aero.Z = Z_total;
aero.L = L_total;
aero.M = M_total;
aero.N = N_total;

aero.alpha = alpha;
aero.beta = beta;
aero.V = V;
aero.qbar = qbar;
```

---

## 13. MATLAB Reference Implementation

```matlab
function aero = aero_coefficients_fighter(x,u_ctrl,params)
% aero_coefficients_fighter
% Educational F-22-inspired aerodynamic model.
% This is not real F-22 aerodynamic data.

% State extraction
u = x(1);
v = x(2);
w = x(3);
p = x(4);
q = x(5);
r = x(6);

% Control extraction
de  = u_ctrl(1);
da  = u_ctrl(2);
dr  = u_ctrl(3);
dT  = u_ctrl(4);
dtv = u_ctrl(5);

% Aircraft parameters
rho  = params.rho;
S    = params.S;
b    = params.b;
cbar = params.cbar;
Tmax = params.Tmax;

% Safe airspeed calculation
V = sqrt(u^2 + v^2 + w^2);
V = max(V,1.0);

% Aerodynamic angles
alpha = atan2(w,u);
beta_arg = max(min(v/V,1),-1);
beta = asin(beta_arg);

% Dynamic pressure
qbar = 0.5*rho*V^2;

% Nondimensional rates
p_hat = p*b/(2*V);
q_hat = q*cbar/(2*V);
r_hat = r*b/(2*V);

% Example longitudinal coefficients
CL0 = 0.20;
CL_alpha = 4.80;
CL_qhat = 7.50;
CL_de = 0.35;

CD0 = 0.025;
Kdrag = 0.09;

Cm0 = 0.02;
Cm_alpha = -0.80;
Cm_qhat = -12.0;
Cm_de = -1.10;
Cm_dtv = -0.80;

% Example lateral-directional coefficients
CY_beta = -0.90;
CY_phat = 0.00;
CY_rhat = 0.25;
CY_da = 0.00;
CY_dr = 0.18;

Cl_beta = -0.12;
Cl_phat = -0.55;
Cl_rhat = 0.18;
Cl_da = 0.22;
Cl_dr = 0.02;

Cn_beta = 0.18;
Cn_phat = -0.04;
Cn_rhat = -0.30;
Cn_da = 0.01;
Cn_dr = -0.12;

% Coefficient equations
CL = CL0 + CL_alpha*alpha + CL_qhat*q_hat + CL_de*de;
CD = CD0 + Kdrag*CL^2;

CY = CY_beta*beta + CY_phat*p_hat + CY_rhat*r_hat + CY_da*da + CY_dr*dr;

Cl = Cl_beta*beta + Cl_phat*p_hat + Cl_rhat*r_hat + Cl_da*da + Cl_dr*dr;
Cm = Cm0 + Cm_alpha*alpha + Cm_qhat*q_hat + Cm_de*de + Cm_dtv*dtv;
Cn = Cn_beta*beta + Cn_phat*p_hat + Cn_rhat*r_hat + Cn_da*da + Cn_dr*dr;

% Convert lift and drag to body-axis coefficients
CX = -CD*cos(alpha) + CL*sin(alpha);
CZ = -CD*sin(alpha) - CL*cos(alpha);

% Aerodynamic forces
X_aero = qbar*S*CX;
Y_aero = qbar*S*CY;
Z_aero = qbar*S*CZ;

% Aerodynamic moments
L_aero = qbar*S*b*Cl;
M_aero = qbar*S*cbar*Cm;
N_aero = qbar*S*b*Cn;

% Propulsion model
T = max(min(dT,1),0)*Tmax;
X_T = T*cos(dtv);
Z_T = -T*sin(dtv);

% Optional thrust-vectoring moment arm
if isfield(params,'lT')
    lT = params.lT;
else
    lT = 0;
end

M_T = -lT*T*sin(dtv);

% Total forces and moments
X_total = X_aero + X_T;
Y_total = Y_aero;
Z_total = Z_aero + Z_T;

L_total = L_aero;
M_total = M_aero + M_T;
N_total = N_aero;

% Output structure
aero.CX = CX;
aero.CY = CY;
aero.CZ = CZ;
aero.Cl = Cl;
aero.Cm = Cm;
aero.Cn = Cn;

aero.CL = CL;
aero.CD = CD;

aero.X = X_total;
aero.Y = Y_total;
aero.Z = Z_total;
aero.L = L_total;
aero.M = M_total;
aero.N = N_total;

aero.alpha = alpha;
aero.beta = beta;
aero.V = V;
aero.qbar = qbar;

aero.p_hat = p_hat;
aero.q_hat = q_hat;
aero.r_hat = r_hat;
end
```

---

## 14. Connection to the Nonlinear 6-DOF Model

The aerodynamic model is called inside the nonlinear dynamics function:

```matlab
aero = aero_coefficients_fighter(x,u_ctrl,params);

X = aero.X;
Y = aero.Y;
Z = aero.Z;
L = aero.L;
M = aero.M;
N = aero.N;
```

These forces and moments are then inserted into the nonlinear 6-DOF equations of motion.

The aerodynamic model is therefore the link between:

```text
Aircraft state + control input
```

and:

```text
Forces + moments driving the nonlinear aircraft motion
```

---

## 15. Connection to Trim

A trim condition is a steady flight condition where the aircraft accelerations are approximately zero.

For level steady flight:

$$
\dot{u} \approx 0
$$

$$
\dot{v} \approx 0
$$

$$
\dot{w} \approx 0
$$

$$
\dot{p} \approx 0
$$

$$
\dot{q} \approx 0
$$

$$
\dot{r} \approx 0
$$

The trim solver adjusts variables such as:

$$
\alpha, \quad \delta_e, \quad \delta_T, \quad \delta_{tv}
$$

to balance aerodynamic forces, thrust, moments, weight, and inertial effects.

The aerodynamic coefficient model strongly affects whether the trim solver finds a reasonable equilibrium.

---

## 16. Connection to Numerical Linearization

After trim is found, the nonlinear plant can be linearized numerically.

The nonlinear system is written as:

$$
\dot{x} = f(x,u)
$$

The linearized system is:

$$
\Delta \dot{x} = A\Delta x + B\Delta u
$$

where:

$$
A = \left.\frac{\partial f}{\partial x}\right|_{x_0,u_0}
$$

$$
B = \left.\frac{\partial f}{\partial u}\right|_{x_0,u_0}
$$

Because the aerodynamic coefficients depend on $\alpha$, $\beta$, $\hat{p}$, $\hat{q}$, $\hat{r}$, and the control inputs, they directly determine the stability derivatives contained inside $A$ and $B$.

---

## 17. Connection to MIMO LQR Control

The MIMO LQR controller uses the linearized model:

$$
\Delta \dot{x} = A\Delta x + B\Delta u
$$

The control law is:

$$
\Delta u = -K\Delta x
$$

where:

$$
\Delta u =
\begin{bmatrix}
\Delta \delta_e \\
\Delta \delta_a \\
\Delta \delta_r \\
\Delta \delta_T \\
\Delta \delta_{tv}
\end{bmatrix}
$$

The aerodynamic model determines how each actuator affects the aircraft:

| Actuator | Primary effect | Secondary coupling |
|---|---|---|
| $\delta_e$ | Pitching moment | Lift and vertical acceleration |
| $\delta_a$ | Rolling moment | Yaw coupling |
| $\delta_r$ | Yawing moment | Side force and roll coupling |
| $\delta_T$ | Forward force | Speed and climb response |
| $\delta_{tv}$ | Pitching moment / vertical thrust | Angle-of-attack response |

This is why the project is a true MIMO control problem.

---

## 18. Model Limitations

The current aerodynamic model has several limitations:

1. The coefficients are representative and educational, not validated aircraft data.
2. The model does not include lookup tables versus Mach number and altitude.
3. The model does not include stall, vortex lift, departure dynamics, or high-angle-of-attack nonlinearities.
4. The model does not include control-surface rate limits unless implemented separately.
5. The thrust-vectoring model is simplified.
6. The model assumes symmetric mass properties and constant geometry.
7. The model does not include landing gear, stores, weapons-bay effects, or compressibility corrections.
8. The model does not represent the real F-22 aerodynamic database.

These limitations are acceptable for the first portfolio version because the objective is to demonstrate flight-dynamics modeling, nonlinear simulation, trim, numerical linearization, and MIMO control design.

---

## 19. Recommended Future Improvements

Future versions of the model can include:

1. Mach-dependent aerodynamic tables.
2. Altitude-dependent atmosphere model.
3. Nonlinear lift curve with stall behavior.
4. Control-surface saturation and rate limits.
5. Separate left and right stabilator commands.
6. Separate left and right thrust-vectoring nozzles.
7. More detailed propulsion dynamics.
8. Wind and turbulence disturbances.
9. Sensor noise and state-estimator integration.
10. Gain-scheduled LQR or nonlinear dynamic inversion.

---

## 20. Summary

This aerodynamic model provides a coefficient-based force and moment representation for the F-22-inspired MIMO 6-DOF fighter simulation.

The model computes:

- airspeed,
- angle of attack,
- sideslip angle,
- dynamic pressure,
- nondimensional angular rates,
- aerodynamic force coefficients,
- aerodynamic moment coefficients,
- body-axis forces,
- body-axis moments,
- simplified thrust and thrust-vectoring effects.

The model is designed to connect directly with:

- nonlinear 6-DOF equations of motion,
- trim calculation,
- numerical linearization,
- MIMO LQR control design,
- closed-loop simulation,
- future autoland and guidance projects.

This document should be stored as:

```text
docs/aerodynamic_model.md
```
