function results = phase3_results_calculator(model, baseline, sf, lqr_ctl, sims, comparison, cfg)
%PHASE3_RESULTS_CALCULATOR Assemble numerical Phase 3 portfolio results.

results = struct;

results.model.A = model.A;
results.model.B = model.B;
results.model.U0 = model.U0;
results.model.h0 = model.h0;
results.model.Kq_phase2 = model.Kq;
results.model.state_names = model.state_names;

results.baseline.open_loop_poles = baseline.poles;
results.baseline.modal_table = baseline.modal.table;
results.baseline.controllability_rank = baseline.controllability_rank;
results.baseline.is_controllable = baseline.is_controllable;

results.recovered_history.phase1_poles = cfg.baseline.phase1_poles;
results.recovered_history.phase2_visible_short_period = ...
    cfg.baseline.phase2_visible_short_period;

if sf.available
    results.state_feedback.available = true;
    results.state_feedback.K = sf.K;
    results.state_feedback.Ntheta = sf.Ntheta;
    results.state_feedback.poles = sf.poles;
    results.state_feedback.modal_table = sf.modal.table;
else
    results.state_feedback.available = false;
    results.state_feedback.reason = sf.reason;
end

results.lqr.Q = lqr_ctl.Q;
results.lqr.R = lqr_ctl.R;
results.lqr.K = lqr_ctl.K;
results.lqr.Ntheta = lqr_ctl.Ntheta;
results.lqr.poles = lqr_ctl.poles;
results.lqr.modal_table = lqr_ctl.modal.table;

if sims.pitch_lqr.available
    m = step_response_metrics(sims.pitch_lqr.theta, ...
                              sims.pitch_lqr.t, ...
                              sims.pitch_lqr.theta_cmd);
    m.peak_pitch_rate_rad_s = max(abs(sims.pitch_lqr.q));
    m.peak_elevator_rad = max(abs(sims.pitch_lqr.delta));
    m.peak_elevator_deg = rad2deg(m.peak_elevator_rad);
    results.pitch_lqr = m;
end

if isfield(sims,'pitch_state_feedback') && sims.pitch_state_feedback.available
    m = step_response_metrics(sims.pitch_state_feedback.theta, ...
                              sims.pitch_state_feedback.t, ...
                              sims.pitch_state_feedback.theta_cmd);
    m.peak_pitch_rate_rad_s = max(abs(sims.pitch_state_feedback.q));
    m.peak_elevator_rad = max(abs(sims.pitch_state_feedback.delta));
    m.peak_elevator_deg = rad2deg(m.peak_elevator_rad);
    results.pitch_state_feedback = m;
end

if sims.altitude_lqr.available
    m = step_response_metrics(sims.altitude_lqr.h, ...
                              sims.altitude_lqr.t, ...
                              sims.altitude_lqr.h_cmd);
    m.peak_pitch_deg = max(abs(rad2deg(sims.altitude_lqr.theta)));
    m.peak_pitch_rate_rad_s = max(abs(sims.altitude_lqr.q));
    m.peak_elevator_deg = max(abs(rad2deg(sims.altitude_lqr.delta)));
    results.altitude_lqr = m;
else
    results.altitude_lqr.available = false;
    results.altitude_lqr.reason = sims.altitude_lqr.reason;
end

if sims.disturbance_lqr.available
    tol = deg2rad(cfg.disturbance.theta_recovery_tolerance_deg);
    theta = sims.disturbance_lqr.theta;
    t = sims.disturbance_lqr.t;

    idx = find(abs(theta) > tol,1,'last');

    if isempty(idx)
        recovery_time = 0;
    elseif idx == numel(t)
        recovery_time = NaN;
    else
        recovery_time = t(idx+1);
    end

    results.disturbance_lqr.maximum_theta_deg = ...
        max(abs(rad2deg(theta)));
    results.disturbance_lqr.recovery_time_s = recovery_time;
    results.disturbance_lqr.peak_elevator_deg = ...
        max(abs(rad2deg(sims.disturbance_lqr.delta)));
    results.disturbance_lqr.final_theta_error_deg = ...
        abs(rad2deg(theta(end)));
end

results.comparison = comparison.summary;

fprintf('\n--- PHASE 3 RESULT SUMMARY ---\n');
fprintf('Controllability rank: %d / %d\n', ...
    baseline.controllability_rank, model.n);

fprintf('LQR gain K:\n');
disp(lqr_ctl.K);

if isfield(results,'pitch_lqr')
    fprintf('LQR pitch settling time: %.6g s\n', ...
        results.pitch_lqr.settling_time);
    fprintf('LQR pitch overshoot: %.6g %%\n', ...
        results.pitch_lqr.overshoot_percent);
    fprintf('LQR peak elevator: %.6g deg\n', ...
        results.pitch_lqr.peak_elevator_deg);
end

if isfield(results,'altitude_lqr') && ...
        isfield(results.altitude_lqr,'available') && ...
        ~results.altitude_lqr.available
    fprintf('Altitude result not yet populated: %s\n', ...
        results.altitude_lqr.reason);
end

end
