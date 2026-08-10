function lqr_ctl = lqr_design(model, cfg)
%LQR_DESIGN Linear Quadratic Regulator for the Phase 3 longitudinal plant.
%
% Controller:
%   delta_e = -K_LQR*x + Ntheta*theta_c
%
% Cost:
%   J = integral( x''Qx + delta_e''R*delta_e ) dt
%
% The default Q/R in phase3_config.m are INITIAL tuning normalizations,
% explicitly not claimed Cessna 182 physical limits.

A = model.A;
Be = model.Be;
n = size(A,1);

Q = cfg.lqr.Q;
R = cfg.lqr.R;

if ~isequal(size(Q), [n n])
    error('LQR Q must be %d-by-%d.', n, n);
end

if ~isscalar(R) || ~isfinite(R) || R <= 0
    error('For the current SISO elevator design, LQR R must be positive scalar.');
end

if rank(ctrb(A,Be)) < n
    error('Cannot synthesize the requested LQR controller: plant is not controllable.');
end

[K,S,poles] = lqr(A, Be, Q, R);
Acl = A - Be*K;

Ctheta = zeros(1,n);
Ctheta(cfg.idx.theta) = 1;
Ntheta = reference_prefilter(A, Be, K, Ctheta);

lqr_ctl.available = true;
lqr_ctl.K = K;
lqr_ctl.S = S;
lqr_ctl.Q = Q;
lqr_ctl.R = R;
lqr_ctl.Acl = Acl;
lqr_ctl.poles = poles;
lqr_ctl.modal = modal_metrics(poles);
lqr_ctl.Ctheta = Ctheta;
lqr_ctl.Ntheta = Ntheta;

fprintf('\n--- LQR DESIGN ---\n');
fprintf('Q =\n');
disp(Q);
fprintf('R =\n');
disp(R);
fprintf('K_LQR =\n');
disp(K);
fprintf('Ntheta = %.10g\n', Ntheta);
fprintf('Closed-loop poles:\n');
disp(poles);

end
