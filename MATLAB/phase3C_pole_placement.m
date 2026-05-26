% ============================================================
% Phase 3C - Pole Placement Control
% Cessna 182 Longitudinal Model
%
% Standard state feedback:
%   delta_e = -Kpp*x + Nbar*r
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
% 2) Controllability
% ------------------------------------------------------------
Co = ctrb(A,B);
disp('--------------------------------------------');
disp('Phase 3C1: Pole Placement');
disp(['Controllability rank = ', num2str(rank(Co))]);

%% ------------------------------------------------------------
% 3) Desired poles
% ------------------------------------------------------------
%desired_poles = [-2.5+2.0j, -2.5-2.0j, -0.08+0.18j, -0.08-0.18j];
desired_poles = [-1.5+1.2j, -1.5-1.2j, -0.05+0.12j, -0.05-0.12j];
Kpp = place(A,B,desired_poles);
Acl_pp = A - B*Kpp;

%% ------------------------------------------------------------
% 4) Reference tracking for theta
% ------------------------------------------------------------
Ctheta = [0 0 0 1];
Nbar = -1 / (Ctheta * (Acl_pp \ B));


sys_pp = ss(Acl_pp, B*Nbar, C, zeros(4,1));

%% ------------------------------------------------------------
% 5) Command input
% ------------------------------------------------------------
theta_cmd_deg = 5;
theta_cmd = deg2rad(theta_cmd_deg);

%t = linspace(0,20,3000);
t = linspace(0,80,6000);
u_cmd = theta_cmd * ones(size(t));

%% ------------------------------------------------------------
% 6) Simulate
% ------------------------------------------------------------
[y,t] = lsim(sys_pp, u_cmd, t, x0);

%% ------------------------------------------------------------
% 7) Extract states
% ------------------------------------------------------------
u_resp     = y(:,1);              % ft/s
alpha_resp = rad2deg(y(:,2));     % deg
q_resp     = rad2deg(y(:,3));     % deg/s
theta_resp = rad2deg(y(:,4));     % deg

theta_cmd_plot = theta_cmd_deg * ones(size(t));

%% ------------------------------------------------------------
% 8) Elevator demand
% delta_e = -Kpp*x + Nbar*theta_cmd
% ------------------------------------------------------------
delta_e = - (Kpp * y')' + Nbar*theta_cmd;    % rad
delta_e_deg = rad2deg(delta_e);

%% ------------------------------------------------------------
% 9) Poles
% ------------------------------------------------------------
poles_pp = eig(Acl_pp);

disp('Pole-placement feedback gain Kpp:');
disp(Kpp);
disp('Closed-loop poles:');
disp(poles_pp);

%% ------------------------------------------------------------
% 10) Theta tracking
% ------------------------------------------------------------
figure('Color','w');
plot(t, theta_resp, 'LineWidth', 1.8); hold on;
plot(t, theta_cmd_plot, '--', 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('\theta (deg)');
title('Phase 3C1: Pole-Placement Pitch-Angle Tracking');
legend('\theta', '\theta_{cmd}', 'Location', 'best');

%% ------------------------------------------------------------
% 11) State responses
% ------------------------------------------------------------
figure('Color','w','Position',[100 100 1200 900]);
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');

nexttile
plot(t,u_resp,'LineWidth',1.5);
grid on;
ylabel('u (ft/s)');
title('Phase 3C1: Pole-Placement State Responses');

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
% 12) Elevator demand
% ------------------------------------------------------------
figure('Color','w');
plot(t, delta_e_deg, 'LineWidth', 1.6);
grid on;
xlabel('Time (s)');
ylabel('\delta_e (deg)');
title('Phase 3C1: Pole-Placement Elevator Demand');

%% ------------------------------------------------------------
% 13) Pole map
% ------------------------------------------------------------
figure('Color','w');
plot(real(eig(A)), imag(eig(A)), 'x', 'MarkerSize', 10, 'LineWidth', 2); hold on;
plot(real(poles_pp), imag(poles_pp), 'o', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
xlabel('Real Axis');
ylabel('Imaginary Axis');
title('Phase 3C1: Open-Loop vs Pole-Placement Poles');
legend('Open-loop','Pole placement','Location','best');