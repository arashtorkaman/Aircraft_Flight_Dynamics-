% ============================================================
% Phase 3A - Corrected Pitch-Attitude Hold
% Cessna 182 Longitudinal Model
%
% States: x = [u alpha q theta]^T
%
% Control law:
%   delta_e = Kq*q + Ktheta*theta - Ntheta*theta_cmd
%
% Closed-loop model:
%   x_dot = (A + B*K)x + B*(-Ntheta)*theta_cmd
% where:
%   K = [0 0 Kq Ktheta]
%
% Ntheta is chosen so theta tracks theta_cmd at steady state.
% ============================================================

clear; clc; close all;

%% ------------------------------------------------------------
% 1) Base longitudinal model
% ------------------------------------------------------------
A = [-0.0456     19.4590     0        -32.1740;
     -0.001314   -2.092534   0.970632   0;
      0.003342  -13.938686  -6.805318   0;
      0          0           1          0];

B = [0;
    -0.202562;
    -34.735884;
     0];

C = eye(4);
D = zeros(4,1);

x0 = [0 0 0 0]';

%% ------------------------------------------------------------
% 2) Gains
% ------------------------------------------------------------
Kq = 12;
Ktheta = 0.8;   % start here, then try 0.3, 0.5, 0.8, 1.0

K = [0 0 Kq Ktheta];

Acl = A + B*K;

%% ------------------------------------------------------------
% 3) Prefilter for theta tracking
% Want theta_ss = theta_cmd
% y = Ctheta*x, with Ctheta = [0 0 0 1]
% x_ss = -Acl\B*(-Ntheta)*theta_cmd = Acl\B*Ntheta*theta_cmd
% so choose Ntheta such that Ctheta*(Acl\B)*Ntheta = 1
% ------------------------------------------------------------
Ctheta = [0 0 0 1];
Ntheta = 1 / (Ctheta * (Acl \ B));

Bcmd = -B * Ntheta;

sys_att = ss(Acl, Bcmd, C, zeros(4,1));

%% ------------------------------------------------------------
% 4) Command input
% ------------------------------------------------------------
theta_cmd_deg = 5;
theta_cmd = deg2rad(theta_cmd_deg);

t = linspace(0,40,4000);
u_cmd = theta_cmd * ones(size(t));

%% ------------------------------------------------------------
% 5) Simulate
% ------------------------------------------------------------
[y,t] = lsim(sys_att, u_cmd, t, x0);

%% ------------------------------------------------------------
% 6) Extract states
% ------------------------------------------------------------
u_resp     = y(:,1);          % ft/s
alpha_resp = rad2deg(y(:,2)); % deg
q_resp     = rad2deg(y(:,3)); % deg/s
theta_resp = rad2deg(y(:,4)); % deg

theta_cmd_plot = theta_cmd_deg * ones(size(t));

%% ------------------------------------------------------------
% 7) Elevator demand
% delta_e = Kq*q + Ktheta*theta - Ntheta*theta_cmd
% ------------------------------------------------------------
delta_e = Kq*y(:,3) + Ktheta*y(:,4) - Ntheta*theta_cmd;   % rad
delta_e_deg = rad2deg(delta_e);

%% ------------------------------------------------------------
% 8) Poles
% ------------------------------------------------------------
poles_att = eig(Acl);

disp('--------------------------------------------');
disp('Corrected Phase 3A: Pitch-Attitude Hold');
disp(['Kq     = ', num2str(Kq)]);
disp(['Ktheta = ', num2str(Ktheta)]);
disp(['Ntheta = ', num2str(Ntheta)]);
disp('Closed-loop poles:');
disp(poles_att);

%% ------------------------------------------------------------
% 9) Theta tracking
% ------------------------------------------------------------
figure('Color','w');
plot(t, theta_resp, 'LineWidth', 1.8); hold on;
plot(t, theta_cmd_plot, '--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('\theta (deg)');
title(['Corrected Pitch-Attitude Hold, K_q = ', num2str(Kq), ...
       ', K_\theta = ', num2str(Ktheta)]);
legend('\theta response', '\theta_{cmd}', 'Location', 'best');

%% ------------------------------------------------------------
% 10) Full state responses
% ------------------------------------------------------------
figure('Color','w','Position',[100 100 1200 900]);
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');

nexttile
plot(t,u_resp,'LineWidth',1.5);
grid on;
ylabel('u (ft/s)');
title('Corrected Phase 3A: State Responses');

nexttile
plot(t,alpha_resp,'LineWidth',1.5);
grid on;
ylabel('\alpha (deg)');

nexttile
plot(t,q_resp,'LineWidth',1.5);
grid on;
ylabel('q (deg/s)');

nexttile
plot(t,theta_resp,'LineWidth',1.5); hold on;
plot(t,theta_cmd_plot,'--','LineWidth',1.2);
grid on;
ylabel('\theta (deg)');
xlabel('Time (s)');
legend('\theta','\theta_{cmd}','Location','best');

%% ------------------------------------------------------------
% 11) Elevator demand
% ------------------------------------------------------------
figure('Color','w');
plot(t, delta_e_deg, 'LineWidth', 1.6);
grid on;
xlabel('Time (s)');
ylabel('\delta_e (deg)');
title('Corrected Phase 3A: Elevator Demand');

%% ------------------------------------------------------------
% 12) Pole map
% ------------------------------------------------------------
figure('Color','w');
plot(real(eig(A)), imag(eig(A)), 'x', 'MarkerSize', 10, 'LineWidth', 2); hold on;
plot(real(poles_att), imag(poles_att), 'o', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
xlabel('Real Axis');
ylabel('Imaginary Axis');
title('Corrected Phase 3A: Open-Loop vs Closed-Loop Poles');
legend('Open-loop','Closed-loop','Location','best');