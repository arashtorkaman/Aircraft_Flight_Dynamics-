function sim = simulate_disturbance_rejection(model, controller, cfg)
%SIMULATE_DISTURBANCE_REJECTION Closed-loop initial-condition response.

sim.available = false;

if ~controller.available
    sim.reason = 'Controller is unavailable.';
    return
end

A = model.A;
Be = model.Be;
K = controller.K;
Acl = A - Be*K;

t = (0:cfg.sim.dt:cfg.sim.disturbance_t_final).';
x0 = cfg.disturbance.x0(:);

if numel(x0) ~= model.n
    error('Disturbance initial-condition vector must have %d elements.', model.n);
end

sys = ss(Acl, zeros(model.n,1), eye(model.n), zeros(model.n,1));
[x,t] = initial(sys,x0,t);

delta = -(x*K.');

sim.available = true;
sim.t = t;
sim.x = x;
sim.theta = x(:,cfg.idx.theta);
sim.q = x(:,cfg.idx.q);
sim.delta = delta;
sim.controller_name = controller.name;

end
