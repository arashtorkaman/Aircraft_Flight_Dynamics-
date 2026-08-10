function sim = simulate_altitude_autopilot(model, lqr_ctl, cfg)
%SIMULATE_ALTITUDE_AUTOPILOT Cascaded altitude-hold + inner LQR simulation.
%
% Architecture implemented:
%
%   h_c -> PI altitude controller -> theta_c
%       -> LQR pitch/state-feedback controller -> delta_e
%       -> aircraft -> h
%
% The LQR controller replaces the classical inner pitch/q loops; it uses
% full-state feedback and a pitch-reference prefilter.
%
% Altitude kinematics:
%   h_dot ~= U0*theta - w
%
% This simulation is intentionally disabled until the user supplies:
%   - actual Phase 1 U0
%   - altitude PI gains Kp and Ki

sim.available = false;
sim.reason = '';

if ~lqr_ctl.available
    sim.reason = 'LQR controller is unavailable.';
    return
end

if ~isfinite(model.U0)
    sim.reason = 'U0 is missing from the imported Phase 1 model.';
    fprintf('\nAltitude simulation skipped: %s\n', sim.reason);
    return
end

Kp = cfg.altitude.Kp;
Ki = cfg.altitude.Ki;

if ~isfinite(Kp) || ~isfinite(Ki)
    sim.reason = ['Altitude PI gains are not set. Enter cfg.altitude.Kp and ' ...
                  'cfg.altitude.Ki after tuning.'];
    fprintf('\nAltitude simulation skipped: %s\n', sim.reason);
    return
end

A = model.A;
Be = model.Be;
K = lqr_ctl.K;
Ntheta = lqr_ctl.Ntheta;
n = model.n;

t = (0:cfg.sim.dt:cfg.sim.altitude_t_final).';

% z = [x; h; integral(e_h); delta_e]
% delta_e is kept as a state even when actuator dynamics are disabled; when
% disabled it is algebraically overwritten in the ODE derivative logic.
x0 = cfg.sim.x0(:);
if numel(x0) ~= n
    x0 = zeros(n,1);
end

z0 = [x0; 0; 0; 0];

ode = @(tt,z) local_altitude_ode(tt,z,A,Be,K,Ntheta,model.U0,cfg);
[t,z] = ode45(ode,t,z0);

x = z(:,1:n);
h = z(:,n+1);
int_h = z(:,n+2);
delta_state = z(:,n+3);

theta_cmd = zeros(size(t));
delta_cmd = zeros(size(t));
delta = zeros(size(t));

for k = 1:numel(t)
    e_h = cfg.altitude.command_m - h(k);
    theta_cmd(k) = Kp*e_h + Ki*int_h(k);
    theta_cmd(k) = apply_theta_limit(theta_cmd(k),cfg);

    delta_cmd(k) = -K*x(k,:).' + Ntheta*theta_cmd(k);

    if cfg.actuator.enable
        delta(k) = delta_state(k);
    else
        delta(k) = delta_cmd(k);
    end
end

sim.available = true;
sim.t = t;
sim.x = x;
sim.h = h;
sim.h_cmd = cfg.altitude.command_m;
sim.theta = x(:,cfg.idx.theta);
sim.q = x(:,cfg.idx.q);
sim.theta_cmd = theta_cmd;
sim.delta_cmd = delta_cmd;
sim.delta = delta;
sim.integral_h = int_h;
sim.controller_name = 'LQR altitude autopilot';

end

function dz = local_altitude_ode(~, z, A, Be, K, Ntheta, U0, cfg)

n = size(A,1);
x = z(1:n);
h = z(n+1);
int_h = z(n+2);
delta_state = z(n+3);

e_h = cfg.altitude.command_m - h;

theta_cmd_raw = cfg.altitude.Kp*e_h + cfg.altitude.Ki*int_h;
theta_cmd = apply_theta_limit(theta_cmd_raw,cfg);

delta_cmd = -K*x + Ntheta*theta_cmd;

if cfg.actuator.enable
    tau = cfg.actuator.time_constant_s;
    if ~isfinite(tau) || tau <= 0
        error('Actuator is enabled but time_constant_s is invalid.');
    end

    delta_limited = apply_actuator_limit(delta_cmd,cfg);
    delta_dot = (delta_limited - delta_state)/tau;
    delta = delta_state;
else
    delta = delta_cmd;
    delta_dot = 0;
end

xdot = A*x + Be*delta;

w = x(cfg.idx.w);
theta = x(cfg.idx.theta);
h_dot = U0*theta - w;

% Basic conditional anti-windup for theta-command saturation.
if isfinite(cfg.altitude.theta_command_limit_deg)
    limit = deg2rad(cfg.altitude.theta_command_limit_deg);
    saturated_high = theta_cmd_raw > limit && e_h > 0;
    saturated_low  = theta_cmd_raw < -limit && e_h < 0;

    if saturated_high || saturated_low
        int_h_dot = 0;
    else
        int_h_dot = e_h;
    end
else
    int_h_dot = e_h;
end

dz = [xdot; h_dot; int_h_dot; delta_dot];

end

function theta_cmd = apply_theta_limit(theta_cmd,cfg)

if isfinite(cfg.altitude.theta_command_limit_deg)
    lim = deg2rad(cfg.altitude.theta_command_limit_deg);
    theta_cmd = min(max(theta_cmd,-lim),lim);
end

end

function u = apply_actuator_limit(u,cfg)

if isfinite(cfg.actuator.position_limit_deg)
    lim = deg2rad(cfg.actuator.position_limit_deg);
    u = min(max(u,-lim),lim);
end

end
