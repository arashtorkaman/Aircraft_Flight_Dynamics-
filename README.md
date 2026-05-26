# F-22-Inspired Nonlinear 6-DOF Fighter Aircraft Simulation

## Project Overview

This project develops a **nonlinear six-degree-of-freedom (6-DOF) flight dynamics simulation** for a high-performance fighter aircraft inspired by the F-22 Raptor.

The objective is to demonstrate practical aerospace engineering capability in:

- nonlinear rigid-body aircraft dynamics
- body-axis force and moment modeling
- Euler-angle and quaternion attitude kinematics
- nonlinear aerodynamic coefficient modeling
- thrust and simplified thrust-vectoring effects
- simulation-ready MATLAB implementation
- future trim, linearization, and control design

This project is intended for an aerospace engineering portfolio and is **not** a real F-22 flight model.

---

## Important Disclaimer

This simulation uses public F-22 geometric and propulsion reference values only.

The following items are **not real F-22 data**:

- inertia tensor
- aerodynamic coefficient database
- stability and control derivatives
- actuator models
- thrust-vectoring control schedules
- flight-control laws
- mission-system architecture
- avionics architecture

The aerodynamic coefficients, inertia values, control inputs, and thrust-vectoring effects used in this project are **generic engineering estimates** created for educational nonlinear flight simulation.

A suitable professional description is:

> F-22-inspired nonlinear 6-DOF high-performance fighter aircraft simulation.

Avoid describing the project as:

> Real F-22 Raptor flight dynamics model.

---

## Public Reference Aircraft Data

The model uses the following public reference values for geometry and propulsion.

| Parameter | Symbol | Value |
|---|---:|---:|
| Length | $L_f$ | $18.9 \ \text{m}$ |
| Wingspan | $b$ | $13.6 \ \text{m}$ |
| Height | $h$ | $5.1 \ \text{m}$ |
| Wing reference area | $S$ | $78.04 \ \text{m}^2$ |
| Empty mass, approximate | $m_{empty}$ | $19{,}700 \ \text{kg}$ |
| Maximum takeoff mass | $m_{max}$ | $38{,}000 \ \text{kg}$ |
| Internal fuel mass | $m_f$ | $8{,}200 \ \text{kg}$ |
| Engine thrust, each | $T_e$ | $\approx 155.7 \ \text{kN}$ |
| Total maximum thrust | $T_{max}$ | $\approx 311 \ \text{kN}$ |

For simulation, a representative mid-loaded mass is assumed:

$$
m = 29{,}000 \ \text{kg}
$$

The approximate mean aerodynamic chord is estimated from:

$$
\bar{c} = \frac{S}{b}
$$

Using:

$$
S = 78.04 \ \text{m}^2
$$

$$
b = 13.6 \ \text{m}
$$

gives:

$$
\bar{c} = \frac{78.04}{13.6} = 5.74 \ \text{m}
$$

---

## Simulation Constants

```matlab
m     = 29000;        % kg, assumed combat-loaded mass
S     = 78.04;        % m^2, wing reference area
b     = 13.6;         % m, wingspan
cbar  = S/b;          % m, approximate mean aerodynamic chord
g     = 9.80665;      % m/s^2
Tmax  = 2*155700;     % N, two engines, afterburning thrust class
```

---

## Estimated Inertia Tensor

The real F-22 inertia tensor is not publicly available. The following values are engineering estimates for a generic fighter model:

$$
I_x = 25{,}000 \ \text{kg m}^2
$$

$$
I_y = 145{,}000 \ \text{kg m}^2
$$

$$
I_z = 165{,}000 \ \text{kg m}^2
$$

$$
I_{xz} = 5{,}000 \ \text{kg m}^2
$$

MATLAB implementation:

```matlab
Ix  = 2.5e4;      % kg*m^2, estimated
Iy  = 1.45e5;     % kg*m^2, estimated
Iz  = 1.65e5;     % kg*m^2, estimated
Ixz = 5.0e3;      % kg*m^2, estimated
```

For the first implementation, $I_{xz}$ may be neglected to simplify the rotational equations. It can be added later for a more complete asymmetric rigid-body model.

---

## State Vector

The nonlinear aircraft state vector is defined as:

$$
x =
\begin{bmatrix}
u & v & w & p & q & r & \phi & \theta & \psi & x_E & y_E & z_E
\end{bmatrix}^T
$$

where:

| State | Description |
|---|---|
| $u$ | body-axis forward velocity |
| $v$ | body-axis lateral velocity |
| $w$ | body-axis vertical velocity |
| $p$ | roll rate |
| $q$ | pitch rate |
| $r$ | yaw rate |
| $\phi$ | roll angle |
| $\theta$ | pitch angle |
| $\psi$ | yaw angle |
| $x_E$ | Earth-frame longitudinal position |
| $y_E$ | Earth-frame lateral position |
| $z_E$ | Earth-frame vertical position |

---

## Control Vector

The control vector is:

$$
u_c =
\begin{bmatrix}
\delta_e & \delta_a & \delta_r & \delta_t & \delta_{tv}
\end{bmatrix}^T
$$

where:

| Control | Description |
|---|---|
| $\delta_e$ | elevator / stabilator command |
| $\delta_a$ | aileron command |
| $\delta_r$ | rudder command |
| $\delta_t$ | throttle command |
| $\delta_{tv}$ | simplified pitch-axis thrust-vectoring command |

---

## Flight Condition Variables

Airspeed:

$$
V = \sqrt{u^2 + v^2 + w^2}
$$

Angle of attack:

$$
\alpha = \tan^{-1}\left(\frac{w}{u}\right)
$$

Sideslip angle:

$$
\beta = \sin^{-1}\left(\frac{v}{V}\right)
$$

Dynamic pressure:

$$
\bar{q} = \frac{1}{2}\rho V^2
$$

---

## Body-Axis Aerodynamic Forces

Aerodynamic forces are computed as:

$$
X_a = \bar{q} S C_X
$$

$$
Y_a = \bar{q} S C_Y
$$

$$
Z_a = \bar{q} S C_Z
$$

where:

- $C_X$ is the body-axis axial-force coefficient
- $C_Y$ is the body-axis side-force coefficient
- $C_Z$ is the body-axis normal-force coefficient

---

## Total Body-Axis Forces with Thrust Vectoring

The total body-axis forces are modeled as:

$$
X = X_a + T \cos(\delta_{tv})
$$

$$
Y = Y_a
$$

$$
Z = Z_a - T \sin(\delta_{tv})
$$

where $T$ is total thrust and $\delta_{tv}$ is a simplified thrust-vectoring angle.

This captures a simplified pitch-axis thrust-vectoring effect and is not intended to reproduce the real F-22 propulsion-control system.

---

## Translational Equations of Motion

The nonlinear translational equations in body axes are:

$$
\dot{u} = rv - qw - g\sin\theta + \frac{X}{m}
$$

$$
\dot{v} = pw - ru + g\sin\phi\cos\theta + \frac{Y}{m}
$$

$$
\dot{w} = qu - pv + g\cos\phi\cos\theta + \frac{Z}{m}
$$

These equations describe the acceleration of the aircraft center of mass in body coordinates.

---

## Aerodynamic Moments

Aerodynamic moments are computed from nondimensional moment coefficients:

$$
L = \bar{q} S b C_l
$$

$$
M = \bar{q} S \bar{c} C_m
$$

$$
N = \bar{q} S b C_n
$$

where:

- $L$ is rolling moment
- $M$ is pitching moment
- $N$ is yawing moment
- $C_l$ is rolling-moment coefficient
- $C_m$ is pitching-moment coefficient
- $C_n$ is yawing-moment coefficient

---

## Simplified Thrust-Vectoring Pitch Moment

A simplified pitch moment from thrust vectoring is modeled as:

$$
M_{tv} = x_T T \sin(\delta_{tv})
$$

where $x_T$ is the longitudinal moment arm between the thrust line and center of gravity.

A representative estimate is:

$$
x_T = 2.0 \ \text{m}
$$

The total pitching moment becomes:

$$
M_{total} = M + M_{tv}
$$

---

## Rotational Equations of Motion

For the first implementation, neglecting $I_{xz}$ gives:

$$
\dot{p} = \frac{L + (I_y - I_z)qr}{I_x}
$$

$$
\dot{q} = \frac{M_{total} + (I_z - I_x)pr}{I_y}
$$

$$
\dot{r} = \frac{N + (I_x - I_y)pq}{I_z}
$$

These equations are suitable for an initial nonlinear 6-DOF simulation.

A future version can include full cross-inertia coupling with $I_{xz}$.

---

## Euler-Angle Attitude Kinematics

Euler-angle kinematics are:

$$
\dot{\phi} = p + q\sin\phi\tan\theta + r\cos\phi\tan\theta
$$

$$
\dot{\theta} = q\cos\phi - r\sin\phi
$$

$$
\dot{\psi} = q\sin\phi\sec\theta + r\cos\phi\sec\theta
$$

Euler angles are intuitive and useful for early simulation and plotting.

However, they suffer from a singularity when:

$$
\theta = \pm 90^\circ
$$

For aggressive fighter maneuvers, quaternion kinematics are preferred.

---

## Quaternion Attitude Kinematics

For advanced simulation, define the quaternion:

$$
q_B^E =
\begin{bmatrix}
q_0 & q_1 & q_2 & q_3
\end{bmatrix}^T
$$

The quaternion kinematic equations are:

$$
\dot{q}_0 = -\frac{1}{2}(q_1p + q_2q + q_3r)
$$

$$
\dot{q}_1 = \frac{1}{2}(q_0p + q_2r - q_3q)
$$

$$
\dot{q}_2 = \frac{1}{2}(q_0q + q_3p - q_1r)
$$

$$
\dot{q}_3 = \frac{1}{2}(q_0r + q_1q - q_2p)
$$

After numerical integration, the quaternion should be normalized:

$$
q \leftarrow \frac{q}{\|q\|}
$$

---

## Nonlinear Longitudinal Aerodynamic Model

The nonlinear lift coefficient is modeled as:

$$
C_L =
C_{L0}
+ C_{L\alpha}\alpha
+ C_{L\alpha3}\alpha^3
+ C_{Lq}\frac{q\bar{c}}{2V}
+ C_{L\delta_e}\delta_e
$$

The drag coefficient is modeled as:

$$
C_D =
C_{D0}
+ K C_L^2
+ C_{D\beta}\beta^2
+ C_{D\delta_e}\delta_e^2
$$

The pitching-moment coefficient is modeled as:

$$
C_m =
C_{m0}
+ C_{m\alpha}\alpha
+ C_{m\alpha3}\alpha^3
+ C_{mq}\frac{q\bar{c}}{2V}
+ C_{m\delta_e}\delta_e
$$

Example coefficient values:

```matlab
CL0       = 0.05;
CL_alpha  = 3.8;      % per rad
CL_alpha3 = -4.5;     % nonlinear stall softening
CL_q      = 7.5;
CL_de     = 0.35;

CD0       = 0.025;
K         = 0.08;
CD_beta   = 0.20;
CD_de     = 0.02;

Cm0       = 0.02;
Cm_alpha  = -0.80;
Cm_alpha3 = 0.60;
Cm_q      = -12.0;
Cm_de     = -1.10;
```

---

## Conversion from Wind-Axis to Body-Axis Coefficients

The body-axis force coefficients are computed from lift and drag using:

$$
C_X = C_L\sin\alpha - C_D\cos\alpha
$$

$$
C_Z = -C_L\cos\alpha - C_D\sin\alpha
$$

These equations allow the aerodynamic force model to be inserted directly into the body-axis equations of motion.

---

## Lateral-Directional Aerodynamic Model

The side-force coefficient is:

$$
C_Y =
C_{Y\beta}\beta
+ C_{Yp}\frac{pb}{2V}
+ C_{Yr}\frac{rb}{2V}
+ C_{Y\delta_a}\delta_a
+ C_{Y\delta_r}\delta_r
$$

The rolling-moment coefficient is:

$$
C_l =
C_{l\beta}\beta
+ C_{lp}\frac{pb}{2V}
+ C_{lr}\frac{rb}{2V}
+ C_{l\delta_a}\delta_a
+ C_{l\delta_r}\delta_r
$$

The yawing-moment coefficient is:

$$
C_n =
C_{n\beta}\beta
+ C_{np}\frac{pb}{2V}
+ C_{nr}\frac{rb}{2V}
+ C_{n\delta_a}\delta_a
+ C_{n\delta_r}\delta_r
$$

Example coefficient values:

```matlab
CY_beta = -0.80;
CY_p    = 0.00;
CY_r    = 0.25;
CY_da   = 0.02;
CY_dr   = 0.25;

Cl_beta = -0.12;
Cl_p    = -0.55;
Cl_r    = 0.18;
Cl_da   = 0.08;
Cl_dr   = 0.02;

Cn_beta = 0.18;
Cn_p    = -0.04;
Cn_r    = -0.25;
Cn_da   = 0.01;
Cn_dr   = -0.10;
```

These values are generic fighter-style coefficients and should be tuned during simulation.

---

## MATLAB Parameter File

The following function can be saved as:

```text
fighter_params.m
```

```matlab
function P = fighter_params()

% F-22-inspired nonlinear fighter model
% Public geometry/mass references are based on open F-22 data.
% Inertia and aerodynamic coefficients are engineering estimates.
% This is NOT a real F-22 flight model.

P.g = 9.80665;

% Geometry
P.S = 78.04;             % m^2
P.b = 13.6;              % m
P.cbar = P.S/P.b;        % m

% Mass
P.m = 29000;             % kg, assumed mid-loaded mass

% Inertia estimates
P.Ix  = 2.5e4;           % kg*m^2
P.Iy  = 1.45e5;          % kg*m^2
P.Iz  = 1.65e5;          % kg*m^2
P.Ixz = 5.0e3;           % kg*m^2

% Propulsion
P.Tmax = 2*155700;       % N, two engines
P.xT = 2.0;              % m, thrust-vector moment arm estimate

% Atmosphere
P.rho = 1.225;           % kg/m^3, sea-level default

% Longitudinal aero coefficients
P.CL0       = 0.05;
P.CL_alpha  = 3.8;
P.CL_alpha3 = -4.5;
P.CL_q      = 7.5;
P.CL_de     = 0.35;

P.CD0       = 0.025;
P.K         = 0.08;
P.CD_beta   = 0.20;
P.CD_de     = 0.02;

P.Cm0       = 0.02;
P.Cm_alpha  = -0.80;
P.Cm_alpha3 = 0.60;
P.Cm_q      = -12.0;
P.Cm_de     = -1.10;

% Lateral-directional aero coefficients
P.CY_beta = -0.80;
P.CY_p    = 0.00;
P.CY_r    = 0.25;
P.CY_da   = 0.02;
P.CY_dr   = 0.25;

P.Cl_beta = -0.12;
P.Cl_p    = -0.55;
P.Cl_r    = 0.18;
P.Cl_da   = 0.08;
P.Cl_dr   = 0.02;

P.Cn_beta = 0.18;
P.Cn_p    = -0.04;
P.Cn_r    = -0.25;
P.Cn_da   = 0.01;
P.Cn_dr   = -0.10;

end
```

---

## Suggested Repository Structure

```text
f22-inspired-nonlinear-6dof/
│
├── README.md
│
├── docs/
│   ├── theory_notes.md
│   ├── model_assumptions.md
│   └── validation_plan.md
│
├── matlab/
│   ├── fighter_params.m
│   ├── aero_coefficients.m
│   ├── eom_6dof_euler.m
│   ├── eom_6dof_quaternion.m
│   ├── run_open_loop_sim.m
│   ├── run_trim_case.m
│   └── plot_results.m
│
├── results/
│   ├── figures/
│   └── simulation_outputs/
│
└── references/
    └── public_reference_links.md
```

---

## Suggested Development Phases

### Phase 1 — Nonlinear Equations of Motion

Implement the nonlinear 6-DOF equations using Euler angles.

Deliverables:

- `fighter_params.m`
- `eom_6dof_euler.m`
- open-loop time simulation
- plots of velocity, attitude, angular rates, and trajectory

---

### Phase 2 — Aerodynamic Coefficient Model

Implement nonlinear aerodynamic coefficients.

Deliverables:

- `aero_coefficients.m`
- plots of $C_L$, $C_D$, and $C_m$ versus angle of attack
- lateral-directional coefficient verification

---

### Phase 3 — Trim and Linearization

Compute steady-level trim and linearize the nonlinear model:

$$
\dot{x} = f(x,u)
$$

around a trim condition:

$$
x_0, u_0
$$

to obtain:

$$
\Delta \dot{x} = A\Delta x + B\Delta u
$$

Deliverables:

- trim condition table
- numerical $A$ and $B$ matrices
- eigenvalue analysis
- short-period, phugoid, Dutch-roll, roll, and spiral mode identification

---

### Phase 4 — Control Design

Design basic control laws:

- pitch-rate damper
- angle-of-attack limiter
- roll-rate command system
- yaw damper
- altitude or flight-path-angle hold

Example pitch-rate damper:

$$
\delta_e = -K_q q
$$

Deliverables:

- open-loop versus closed-loop comparison
- control surface history
- stability improvement plots
- engineering interpretation

---

### Phase 5 — Advanced Fighter Simulation

Add advanced features:

- quaternion-based attitude propagation
- actuator saturation
- actuator rate limits
- simplified thrust vectoring
- aggressive maneuver simulation
- 3D trajectory visualization

Deliverables:

- quaternion simulation
- high-angle-of-attack maneuver case
- trajectory and attitude plots
- limitations and assumptions section

---

## Example Output Plots

Recommended plots for the portfolio:

1. Body-axis velocities: $u$, $v$, $w$
2. Angular rates: $p$, $q$, $r$
3. Euler angles: $\phi$, $\theta$, $\psi$
4. Angle of attack: $\alpha$
5. Sideslip angle: $\beta$
6. Control inputs: $\delta_e$, $\delta_a$, $\delta_r$, $\delta_t$, $\delta_{tv}$
7. Aerodynamic coefficients: $C_L$, $C_D$, $C_m$, $C_Y$, $C_l$, $C_n$
8. 3D flight trajectory
9. Open-loop versus closed-loop response
10. Trim and linearization eigenvalue plot

---

## Engineering Value of This Project

This project demonstrates the ability to move from classical flight mechanics into full nonlinear aircraft simulation.

It shows competency in:

- nonlinear aircraft equations of motion
- dynamic pressure and aerodynamic force modeling
- stability and control derivatives
- rigid-body rotational dynamics
- attitude kinematics
- MATLAB simulation structure
- control-oriented modeling
- aerospace portfolio documentation

This is a strong advanced portfolio project because it connects aircraft dynamics, simulation, controls, and professional documentation.

---

## Limitations

This model is not suitable for:

- real aircraft performance prediction
- F-22 flight-envelope analysis
- classified or operational control-law reconstruction
- weapons-system modeling
- mission-system simulation
- flight-test correlation

It is suitable for:

- educational nonlinear dynamics simulation
- portfolio demonstration
- control design practice
- trim and linearization studies
- MATLAB/Simulink aerospace modeling practice

---

## References

Suggested public references to cite in project documentation:

1. U.S. Air Force public F-22 fact sheet  
2. NASA and university flight dynamics textbooks  
3. Stevens, Lewis, and Johnson — *Aircraft Control and Simulation*  
4. Etkin and Reid — *Dynamics of Flight*  
5. Nelson — *Flight Stability and Automatic Control*  
6. Zipfel — *Modeling and Simulation of Aerospace Vehicle Dynamics*

---

## Recommended Project Title

Use one of the following titles:

```text
F-22-Inspired Nonlinear 6-DOF Fighter Aircraft Simulation
```

or:

```text
Nonlinear 6-DOF Flight Dynamics and Control of a Generic Fifth-Generation Fighter Aircraft
```

Avoid:

```text
Real F-22 Raptor Flight Dynamics Model
```

---

## Professional README Statement

This project uses public reference dimensions and propulsion data from the F-22 Raptor as inspiration for a generic fifth-generation fighter aircraft simulation. The objective is to demonstrate nonlinear 6-DOF aircraft modeling, aerodynamic force and moment construction, attitude kinematics, and control-oriented simulation. The model does not contain real F-22 aerodynamic data, control laws, inertia properties, or classified information.
