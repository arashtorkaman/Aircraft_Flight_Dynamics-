# Phase 3 — Autopilot and Flight Control

This folder contains the Phase 3 longitudinal autopilot and modern flight-control portion of the Aircraft Flight Dynamics portfolio.

## Phase Objective

Phase 3 extends the Phase 1 longitudinal aircraft model and the Phase 2 pitch-rate damper into full-state feedback, LQR control, command tracking, and a cascaded longitudinal autopilot architecture.

The progression is:

`Phase 1 open-loop aircraft → Phase 2 pitch-rate damper → Phase 3 state feedback / LQR → cascaded autopilot`

## Repository Structure

```text
phase3_autopilot_and_flight_control/
├── README.md
├── docs/
│   ├── theory.md
│   ├── state_feedback.md
│   ├── lqr_design.md
│   ├── autopilot_architecture.md
│   ├── controller_tuning.md
│   ├── phase3_results.md
│   ├── phase3_baseline_from_phase1_phase2.md
│   └── dev_notes/
│       └── github_math_rendering_test.md
├── matlab/
│   ├── run_phase3.m
│   ├── phase3_config.m
│   ├── aircraft_model.m
│   ├── state_feedback_design.m
│   ├── lqr_design.m
│   ├── autopilot_simulation.m
│   ├── controller_comparison.m
│   ├── phase3_plots.m
│   └── phase3_results_calculator.m
├── simulink/
│   └── README.md
└── figures/
    └── simulink_state_feedback_and_autopilot_architecture.png
```

## Documentation

### 1. [Theory](docs/theory.md)

Defines the Phase 3 mathematical foundation: longitudinal state-space dynamics, closed-loop state feedback, command tracking, LQR theory, cascaded-loop structure, performance metrics, actuator constraints, and verification philosophy.

### 2. [State-Feedback Control](docs/state_feedback.md)

Covers the control law

$$\delta_e=-Kx$$

and the closed-loop system

$$A_{cl}=A-BK$$

including controllability, pole placement, state measurement, and reference tracking.

### 3. [LQR Design](docs/lqr_design.md)

Documents the Linear Quadratic Regulator formulation

$$J=\int_0^\infty\left(x^TQx+u^TRu\right)dt$$

with state/control weighting, Riccati solution, tuning methodology, actuator limits, and robustness checks.

### 4. [Autopilot Architecture](docs/autopilot_architecture.md)

Defines the cascaded longitudinal autopilot:

$$h_c\rightarrow\theta_c\rightarrow q_c\rightarrow\delta_e$$

and documents the corresponding Simulink architecture.

### 5. [Controller Tuning](docs/controller_tuning.md)

Defines the tuning workflow for the pitch-rate, attitude, altitude, and LQR control loops, including objective performance metrics and actuator verification.

### 6. [Phase 3 Results](docs/phase3_results.md)

Contains the verification/reporting structure for open-loop poles, controllability, LQR results, tracking performance, control effort, disturbance rejection, and MATLAB/Simulink comparison.

### 7. [Phase 1/2 Baseline](docs/phase3_baseline_from_phase1_phase2.md)

Carries the previously obtained Phase 1 and Phase 2 aircraft dynamics results into Phase 3 so the controller design remains traceable to the earlier phases.

## MATLAB

`matlab/phase3_results_calculator.m` calculates the numerical quantities used to populate `docs/phase3_results.md`, including:

- open-loop poles,
- natural frequencies,
- damping ratios,
- controllability rank,
- pole-placement gains,
- LQR gains,
- pitch tracking metrics,
- altitude tracking metrics after state augmentation,
- elevator demand,
- disturbance-rejection metrics.

The script intentionally does not invent the Phase 1 `A` and `B` matrices. Run the Phase 1 aircraft-model script first so those matrices exist in the MATLAB workspace.

## Simulink

The `simulink/` folder is reserved for the Phase 3 `.slx` models. See `simulink/README.md` for the intended model hierarchy.

## Figures

The `figures/` folder contains architecture diagrams and should later contain simulation outputs such as:

- `altitude_response.png`
- `pitch_response.png`
- `pitch_rate_response.png`
- `elevator_command.png`
- `closed_loop_poles.png`

## Phase 3 Completion Criteria

Phase 3 is complete when the repository demonstrates:

1. traceability to the Phase 1 aircraft model,
2. controllability analysis,
3. state-feedback and/or LQR synthesis,
4. stable closed-loop poles,
5. pitch command tracking,
6. altitude command tracking using an augmented altitude state,
7. physically reasonable elevator demand,
8. controller tuning rationale,
9. MATLAB and Simulink agreement,
10. documented assumptions, limitations, and transition to Phase 4 nonlinear 6-DOF dynamics.
