# Phase 3 MATLAB

This directory contains the MATLAB implementation for:

1. Phase 1 plant reuse and verification
2. Phase 2 baseline traceability
3. full-state feedback / pole placement
4. LQR synthesis
5. pitch-angle command tracking
6. altitude-state augmentation
7. cascaded altitude-hold + inner LQR autopilot
8. disturbance rejection
9. controller comparison
10. plot generation
11. numerical result export

## Required MATLAB toolbox

The controller-design files use Control System Toolbox functions such as:

- `ss`
- `ctrb`
- `place`
- `lqr`
- `lsim`
- `initial`
- `stepinfo`

## First-time setup

Phase 3 does **not** invent a new aircraft model.

From the MATLAB workspace in which your actual Phase 1/2 variables exist, run:

```matlab
import_phase1_model(A,B,'U0',U0,'h0',h0,'Kq',Kq);
```

If one of `U0`, `h0`, or `Kq` has not been recovered yet, omit it:

```matlab
import_phase1_model(A,B);
```

This creates:

```text
data/phase1_longitudinal_model.mat
```

Then execute:

```matlab
run_phase3
```

## Core files

```text
run_phase3.m
phase3_config.m
aircraft_model.m
load_phase1_model.m
import_phase1_model.m
validate_phase1_baseline.m
modal_metrics.m

state_feedback_design.m
desired_poles_from_modes.m
reference_prefilter.m

lqr_design.m
augment_altitude_model.m

autopilot_simulation.m
simulate_pitch_tracking.m
simulate_altitude_autopilot.m
simulate_disturbance_rejection.m

controller_comparison.m
step_response_metrics.m
phase3_results_calculator.m
phase3_plots.m
export_phase3_results.m
```

## Recovered numerical baseline

The code stores the previously recovered Phase 1 poles as a warning-only validation reference:

```text
-3.2548 + 1.7168i
-3.2548 - 1.7168i
-0.0052 + 0.1461i
-0.0052 - 0.1461i
```

It also records the visible recovered Phase 2 short-period closed-loop pair:

```text
-3.5534 + 2.0426i
-3.5534 - 2.0426i
```

Those pole values do **not** uniquely determine the original `A`, `B`, or `Kq`, so the code never reconstructs them artificially.

## State convention

The Phase 3 documentation and scripts assume:

```text
x = [u w q theta]^T
```

with elevator as the primary longitudinal control input.

## LQR weights

`phase3_config.m` contains initial Bryson-style state/control normalizations solely so the LQR workflow has a reproducible starting point.

They are **design starting values, not sourced aircraft physical limits**. Before final publication, replace them with justified allowable excursions and document the tuning study.

## State-feedback pole placement

Pole-placement design is intentionally disabled until you specify targets.

You can specify explicit poles:

```matlab
cfg.sf.desired_poles = [
    ...
];
```

or modal targets:

```matlab
cfg.sf.targets.wn_short_period = ...;
cfg.sf.targets.zeta_short_period = ...;
cfg.sf.targets.wn_phugoid = ...;
cfg.sf.targets.zeta_phugoid = ...;
```

## Altitude autopilot

The original Phase 1 four-state model has no altitude state. The code uses the documented small-angle augmentation:

```text
h_dot ~= U0*theta - w
```

The altitude autopilot is therefore disabled until:

1. actual trim speed `U0` is supplied, and
2. `cfg.altitude.Kp` and `cfg.altitude.Ki` are tuned.

The implemented hierarchy is:

```text
h_c
 |
 v
PI altitude loop
 |
 v
theta_c
 |
 v
LQR full-state pitch controller
 |
 v
delta_e
 |
 v
Aircraft
```

This is deliberately more internally consistent than trying to use a static LQR prefilter to track a constant nonzero pitch-rate command.

## Generated outputs

`run_phase3.m` writes numerical results to:

```text
results/phase3_results.mat
results/phase3_results_generated.md
```

and figures to:

```text
figures/closed_loop_poles.png
figures/pitch_response.png
figures/pitch_rate_response.png
figures/elevator_command.png
figures/altitude_response.png
figures/altitude_pitch_command.png
```

Altitude figures are generated only after the altitude loop has been configured.
