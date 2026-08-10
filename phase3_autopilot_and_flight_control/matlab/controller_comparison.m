function comparison = controller_comparison(model, baseline, sf, lqr_ctl)
%CONTROLLER_COMPARISON Compare open-loop, Phase 2, state feedback, and LQR.
%
% Phase 2 is included only if the actual Kq was imported from the earlier
% project. The recovered short-period Phase 2 pole pair is not enough to
% reconstruct Kq or the full Phase 2 closed-loop model.

names = {};
pole_sets = {};
stable = [];
dominant_real = [];
min_damping = [];

% Open loop
names{end+1,1} = 'Open loop';
pole_sets{end+1,1} = eig(model.A);
m = modal_metrics(pole_sets{end});
stable(end+1,1) = all(real(pole_sets{end}) < 0);
dominant_real(end+1,1) = max(real(pole_sets{end}));
min_damping(end+1,1) = min(m.zeta(isfinite(m.zeta)));

% Phase 2 pitch-rate damper, if Kq exists
if isfinite(model.Kq) && model.n >= 3
    Cq = zeros(1,model.n);
    Cq(3) = 1;

    A_phase2 = model.A - model.Be*model.Kq*Cq;
    p2 = eig(A_phase2);
    m2 = modal_metrics(p2);

    names{end+1,1} = 'Phase 2 pitch-rate damper';
    pole_sets{end+1,1} = p2;
    stable(end+1,1) = all(real(p2) < 0);
    dominant_real(end+1,1) = max(real(p2));
    min_damping(end+1,1) = min(m2.zeta(isfinite(m2.zeta)));
end

if sf.available
    p = sf.poles;
    m_sf = sf.modal;

    names{end+1,1} = 'Phase 3 state feedback';
    pole_sets{end+1,1} = p;
    stable(end+1,1) = all(real(p) < 0);
    dominant_real(end+1,1) = max(real(p));
    min_damping(end+1,1) = min(m_sf.zeta(isfinite(m_sf.zeta)));
end

if lqr_ctl.available
    p = lqr_ctl.poles;
    m_lqr = lqr_ctl.modal;

    names{end+1,1} = 'Phase 3 LQR';
    pole_sets{end+1,1} = p;
    stable(end+1,1) = all(real(p) < 0);
    dominant_real(end+1,1) = max(real(p));
    min_damping(end+1,1) = min(m_lqr.zeta(isfinite(m_lqr.zeta)));
end

comparison.summary = table(string(names), logical(stable), dominant_real, min_damping, ...
    'VariableNames', {'Controller','Stable','DominantRealPole','MinimumPoleDamping'});

comparison.poles = pole_sets;
comparison.names = names;
comparison.baseline = baseline;

fprintf('\n--- CONTROLLER COMPARISON ---\n');
disp(comparison.summary);

end
