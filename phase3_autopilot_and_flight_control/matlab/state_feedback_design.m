function sf = state_feedback_design(model, cfg)
%STATE_FEEDBACK_DESIGN Full-state pole-placement controller for Phase 3.
%
% Controller:
%   delta_e = -K*x + Ntheta*theta_c

sf.available = false;
sf.reason = '';

A = model.A;
Be = model.Be;
n = size(A,1);

Co = ctrb(A,Be);
if rank(Co) < n
    sf.reason = 'Plant is not fully controllable through the selected elevator input.';
    warning(sf.reason);
    return
end

desired = cfg.sf.desired_poles;

if isempty(desired)
    t = cfg.sf.targets;
    values = [t.wn_short_period, t.zeta_short_period, ...
              t.wn_phugoid, t.zeta_phugoid];

    if all(isfinite(values))
        desired = desired_poles_from_modes( ...
            t.wn_short_period, t.zeta_short_period, ...
            t.wn_phugoid, t.zeta_phugoid);
    end
end

if isempty(desired)
    sf.reason = ['No pole-placement targets were specified. Set either ' ...
                 'cfg.sf.desired_poles or the four cfg.sf.targets values.'];
    fprintf('\n--- STATE FEEDBACK DESIGN ---\n');
    fprintf('%s\n', sf.reason);
    return
end

desired = desired(:);
if numel(desired) ~= n
    error('Number of desired poles (%d) must equal number of states (%d).', ...
          numel(desired), n);
end

K = place(A, Be, desired.');
Acl = A - Be*K;
poles = eig(Acl);

Ctheta = zeros(1,n);
Ctheta(cfg.idx.theta) = 1;
Ntheta = reference_prefilter(A, Be, K, Ctheta);

sf.available = true;
sf.K = K;
sf.Acl = Acl;
sf.desired_poles = desired;
sf.poles = poles;
sf.modal = modal_metrics(poles);
sf.Ntheta = Ntheta;
sf.Ctheta = Ctheta;

fprintf('\n--- STATE FEEDBACK DESIGN ---\n');
fprintf('K =\n');
disp(K);
fprintf('Ntheta = %.10g\n', Ntheta);
fprintf('Closed-loop poles:\n');
disp(poles);

end
