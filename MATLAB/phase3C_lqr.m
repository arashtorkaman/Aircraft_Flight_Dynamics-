% ============================================================
% Phase 3C2 - LQR Control
% Cessna 182 Longitudinal Model
%
% States: x = [u alpha q theta]^T
% Input : delta_e
%
% Control law:
%   delta_e = -Klqr*x + Nbar*theta_cmd
%
% Output tracked:
%   theta
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

x0 = zeros(4,1);

%% ------------------------------------------------------------
% 2) LQR weighting matrices
% Q penalizes state deviations
% R penalizes elevator effort
% ------------------------------------------------------------
%Q = diag([0.01, 20, 5, 30]);
%R = 1;
%Q = diag([0.01, 25, 40, 12]);
%R = 20;

Q = diag([0.01, 20, 45, 20]);
R = 25;

Klqr = lqr(A,B,Q,R);
Acl_lqr = A - B*Klqr;

%% ------------------------------------------------------------
% 3) Reference tracking for theta
% We want theta -> theta_cmd at steady state
% ------------------------------------------------------------
Ctheta = [0 0 0 1];
Nbar = -1 / (Ctheta * (Acl_lqr \ B));

sys_lqr = ss(Acl_lqr, B*Nbar, C, zeros(4,1));

%% ------------------------------------------------------------
% 4) Command input
% ------------------------------------------------------------
theta_cmd_deg = 5;
theta_cmd = deg2rad(theta_cmd_deg);

t = linspace(0,80,6000);
u_cmd = theta_cmd * ones(size(t));

%% ------------------------------------------------------------
% 5) Simulate
% ------------------------------------------------------------
[y,t] = lsim(sys_lqr, u_cmd, t, x0);

%% ------------------------------------------------------------
% 6) Extract states
% ------------------------------------------------------------
u_resp     = y(:,1);              % ft/s
alpha_resp = rad2deg(y(:,2));     % deg
q_resp     = rad2deg(y(:,3));     % deg/s
theta_resp = rad2deg(y(:,4));     % deg

theta_cmd_plot = theta_cmd_deg * ones(size(t));

%% ------------------------------------------------------------
% 7) Elevator demand
% ------------------------------------------------------------
delta_e = - (Klqr * y')' + Nbar*theta_cmd;   % rad
delta_e_deg = rad2deg(delta_e);

%% ------------------------------------------------------------
% 8) Closed-loop poles
% ------------------------------------------------------------
poles_lqr = eig(Acl_lqr);

disp('--------------------------------------------');
disp('Phase 3C2: LQR Control');
disp('LQR gain Klqr = ');
disp(Klqr);
disp(['Nbar = ', num2str(Nbar)]);
disp('Closed-loop poles:');
disp(poles_lqr);

%% ------------------------------------------------------------
% 9) Theta tracking
% ------------------------------------------------------------
figure('Color','w');
plot(t, theta_resp, 'LineWidth', 1.8); hold on;
plot(t, theta_cmd_plot, '--', 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('\theta (deg)');
title('Phase 3C2: LQR Pitch-Angle Tracking');
legend('\theta', '\theta_{cmd}', 'Location', 'best');

%% ------------------------------------------------------------
% 10) State responses
% ------------------------------------------------------------
figure('Color','w','Position',[100 100 1200 900]);
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');

nexttile
plot(t,u_resp,'LineWidth',1.5);
grid on;
ylabel('u (ft/s)');
title('Phase 3C2: LQR State Responses');

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
title('Phase 3C2: LQR Elevator Demand');

%% ------------------------------------------------------------
% 12) Pole map
% ------------------------------------------------------------
figure('Color','w');
plot(real(eig(A)), imag(eig(A)), 'x', 'MarkerSize', 10, 'LineWidth', 2); hold on;
plot(real(poles_lqr), imag(poles_lqr), 'o', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
xlabel('Real Axis');
ylabel('Imaginary Axis');
title('Phase 3C2: Open-Loop vs LQR Closed-Loop Poles');
legend('Open-loop','LQR','Location','best');

%% ------------------------------------------------------------
% 13) Final value check
% ------------------------------------------------------------
disp(['Final theta (deg) = ', num2str(theta_resp(end))]);
disp(['Theta command (deg) = ', num2str(theta_cmd_deg)]);