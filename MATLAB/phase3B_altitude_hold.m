% ============================================================
% Phase 3B - Altitude-Hold Demonstration
% Cessna 182 Longitudinal Model + Altitude Augmentation
%
% Augmented states: x = [u alpha q theta h]^T
%
% Approximate altitude kinematics:
%   h_dot = Vtrim * theta
%
% Control structure:
%   theta_cmd = Kh*(h_cmd - h)
%   delta_e   = Kq*q - Ktheta*theta + Ktheta*theta_cmd
%
% Therefore:
%   delta_e = Kq*q - Ktheta*theta + Ktheta*Kh*(h_cmd - h)
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

%% ------------------------------------------------------------
% 2) Altitude augmentation
% h_dot = Vtrim * theta
% ------------------------------------------------------------
Vtrim = 220.1;  % ft/s

A_aug = [A            zeros(4,1);
         0 0 0 Vtrim  0];

B_aug = [B;
         0];

C_aug = eye(5);
D_aug = zeros(5,1);

x0 = zeros(5,1);

%% ------------------------------------------------------------
% 3) Gains
% ------------------------------------------------------------
Kq = 5;
Ktheta = -1.5;
Kh = 0.005;      % altitude outer-loop gain

% delta_e = Kq*q - Ktheta*theta - Ktheta*Kh*h + Ktheta*Kh*h_cmd
K_aug = [0 0 Kq -Ktheta -Ktheta*Kh];

Acl = A_aug + B_aug*K_aug;
Bcmd = B_aug*(Ktheta*Kh);

sys_h = ss(Acl, Bcmd, C_aug, zeros(5,1));

%% ------------------------------------------------------------
% 4) Altitude command
% ------------------------------------------------------------
h_cmd_ft = 100;                      % step altitude command (ft)

t = linspace(0,100,5000);
u_cmd = h_cmd_ft * ones(size(t));

%% ------------------------------------------------------------
% 5) Simulate
% ------------------------------------------------------------
[y,t] = lsim(sys_h, u_cmd, t, x0);

%% ------------------------------------------------------------
% 6) Extract states
% ------------------------------------------------------------
u_resp     = y(:,1);              % ft/s
alpha_resp = rad2deg(y(:,2));     % deg
q_resp     = rad2deg(y(:,3));     % deg/s
theta_resp = rad2deg(y(:,4));     % deg
h_resp     = y(:,5);              % ft

h_cmd_plot = h_cmd_ft * ones(size(t));

%% ------------------------------------------------------------
% 7) Reconstruct theta_cmd and elevator demand
% ------------------------------------------------------------
theta_cmd = Kh*(h_cmd_ft - h_resp);      % rad
theta_cmd_deg = rad2deg(theta_cmd);

delta_e = Kq*y(:,3) - Ktheta*y(:,4) + Ktheta*theta_cmd;   % rad
delta_e_deg = rad2deg(delta_e);

%% ------------------------------------------------------------
% 8) Closed-loop poles
% ------------------------------------------------------------
poles_h = eig(Acl);

disp('--------------------------------------------');
disp('Phase 3B: Altitude-Hold Demonstration');
disp('Closed-loop poles:');
disp(poles_h);

%% ------------------------------------------------------------
% 9) Plot altitude response
% ------------------------------------------------------------
figure('Color','w');
plot(t, h_resp, 'LineWidth', 1.8); hold on;
plot(t, h_cmd_plot, '--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Altitude h (ft)');
title(['Altitude Hold Response, K_q = ', num2str(Kq), ...
       ', K_\theta = ', num2str(Ktheta), ', K_h = ', num2str(Kh)]);
legend('h response', 'h_{cmd}', 'Location', 'best');

%% ------------------------------------------------------------
% 10) Plot theta response and theta command
% ------------------------------------------------------------
figure('Color','w');
plot(t, theta_resp, 'LineWidth', 1.8); hold on;
plot(t, theta_cmd_deg, '--', 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('\theta (deg)');
title('Phase 3B: Pitch Response Inside Altitude Loop');
legend('\theta', '\theta_{cmd}', 'Location', 'best');

%% ------------------------------------------------------------
% 11) Plot full responses
% ------------------------------------------------------------
figure('Color','w','Position',[100 100 1200 1000]);
tiledlayout(5,1,'TileSpacing','compact','Padding','compact');

nexttile
plot(t,u_resp,'LineWidth',1.4);
grid on;
ylabel('u (ft/s)');
title('Phase 3B: State Responses');

nexttile
plot(t,alpha_resp,'LineWidth',1.4);
grid on;
ylabel('\alpha (deg)');

nexttile
plot(t,q_resp,'LineWidth',1.4);
grid on;
ylabel('q (deg/s)');

nexttile
plot(t,theta_resp,'LineWidth',1.4); hold on;
plot(t,theta_cmd_deg,'--','LineWidth',1.2);
grid on;
ylabel('\theta (deg)');
legend('\theta','\theta_{cmd}','Location','best');

nexttile
plot(t,h_resp,'LineWidth',1.4); hold on;
plot(t,h_cmd_plot,'--','LineWidth',1.2);
grid on;
ylabel('h (ft)');
xlabel('Time (s)');
legend('h','h_{cmd}','Location','best');

%% ------------------------------------------------------------
% 12) Elevator demand
% ------------------------------------------------------------
figure('Color','w');
plot(t, delta_e_deg, 'LineWidth', 1.6);
grid on;
xlabel('Time (s)');
ylabel('\delta_e (deg)');
title('Phase 3B: Elevator Demand');

%% ------------------------------------------------------------
% 13) Pole map
% ------------------------------------------------------------
figure('Color','w');
plot(real(poles_h), imag(poles_h), 'o', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
xlabel('Real Axis');
ylabel('Imaginary Axis');
title('Phase 3B: Closed-Loop Poles');