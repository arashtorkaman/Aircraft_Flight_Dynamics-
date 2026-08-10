function sims = autopilot_simulation(model, sf, lqr_ctl, cfg)
%AUTOPILOT_SIMULATION Run Phase 3 command and disturbance simulations.

sims = struct;

% Label controller structs so plot/results functions can identify them.
if sf.available
    sf.name = 'State feedback';
    sims.pitch_state_feedback = simulate_pitch_tracking(model,sf,cfg);
else
    sims.pitch_state_feedback.available = false;
    sims.pitch_state_feedback.reason = sf.reason;
end

lqr_ctl.name = 'LQR';
sims.pitch_lqr = simulate_pitch_tracking(model,lqr_ctl,cfg);

sims.altitude_lqr = simulate_altitude_autopilot(model,lqr_ctl,cfg);

sims.disturbance_lqr = simulate_disturbance_rejection(model,lqr_ctl,cfg);

end
