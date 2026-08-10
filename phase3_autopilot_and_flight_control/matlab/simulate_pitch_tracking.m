function sim = simulate_pitch_tracking(model, controller, cfg)
%SIMULATE_PITCH_TRACKING Simulate a constant pitch-angle command.
%
% Supported controller structure:
%   delta_e_cmd = -K*x + Ntheta*theta_cmd
%
% If actuator dynamics are disabled, the closed-loop LTI model is simulated
% directly. If enabled, a first-order actuator with optional position
% saturation is included using ode45.

if ~isfield(controller,'available') || ~controller.available
    sim.available = false;
    sim.reason = 'Controller is unavailable.';
    return
end

A = model.A;
Be = model.Be;
K = controller.K;
Ntheta = controller.Ntheta;
n = size(A,1);

theta_cmd = deg2rad(cfg.pitch.theta_cmd_deg);
t = (0:cfg.sim.dt:cfg.sim.pitch_t_final).';

x0 = cfg.sim.x0(:);
if numel(x0) ~= n
    x0 = zeros(n,1);
end

if ~cfg.actuator.enable
    Acl = A - Be*K;
    sys_x = ss(Acl, Be*Ntheta, eye(n), zeros(n,1));

    r = theta_cmd * ones(size(t));
    [x,~,~] = lsim(sys_x, r, t, x0);

    delta_cmd = -(x*K.') + Ntheta*r;
    delta = delta_cmd;
else
    tau = cfg.actuator.time_constant_s;
    if ~isfinite(tau) || tau <= 0
        error('Actuator is enabled but time_constant_s is invalid.');
    end

    z0 = [x0; 0];

    ode = @(tt,z) local_ode(tt,z,A,Be,K,Ntheta,theta_cmd,cfg);
    [t,xz] = ode45(ode, t, z0);

    x = xz(:,1:n);
    delta = xz(:,n+1);

    delta_cmd = zeros(size(t));
    for k = 1:numel(t)
        delta_cmd(k) = -K*x(k,:).' + Ntheta*theta_cmd;
    end
end

sim.available = true;
sim.t = t;
sim.x = x;
sim.theta_cmd = theta_cmd;
sim.theta = x(:,cfg.idx.theta);
sim.q = x(:,cfg.idx.q);
sim.delta_cmd = delta_cmd;
sim.delta = delta;
sim.controller_name = controller.name;

end

function dz = local_ode(~, z, A, Be, K, Ntheta, theta_cmd, cfg)

n = size(A,1);
x = z(1:n);
delta = z(n+1);

delta_cmd = -K*x + Ntheta*theta_cmd;
delta_limited = apply_actuator_limit(delta_cmd, cfg);

tau = cfg.actuator.time_constant_s;
delta_dot = (delta_limited - delta) / tau;

xdot = A*x + Be*delta;

dz = [xdot; delta_dot];

end

function u = apply_actuator_limit(u, cfg)

limit_deg = cfg.actuator.position_limit_deg;

if isfinite(limit_deg)
    limit = deg2rad(limit_deg);
    u = min(max(u,-limit),limit);
end

end
