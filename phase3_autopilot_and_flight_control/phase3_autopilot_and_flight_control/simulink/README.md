# Simulink Models

This folder is reserved for the Phase 3 Simulink implementation.

Recommended model files:

```text
longitudinal_state_feedback.slx
longitudinal_lqr_autopilot.slx
```

## State-Feedback Model

Recommended signal flow:

```text
Reference
   |
   v
Reference / Prefilter
   |
   v
State-Feedback Controller
u = -Kx + Nr
   |
   v
Actuator / Saturation
   |
   v
Longitudinal Aircraft Plant
xdot = Ax + Bu
   |
   v
State Outputs
   |
   +-------------------- feedback --------------------+
```

## Cascaded Autopilot

Recommended signal flow:

```text
Altitude Command h_c
       |
       v
Altitude Loop
       |
       v
Pitch Command theta_c
       |
       v
Pitch / State-Feedback Loop
       |
       v
Elevator Command delta_e
       |
       v
Actuator
       |
       v
Aircraft Plant
```

The implementation should use the same plant matrices, gains, initial conditions, command inputs, saturation limits, and units as the MATLAB verification model.
