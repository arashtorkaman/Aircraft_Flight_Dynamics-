% ============================================================
% Phase 3C2 - LQI / Servo-LQR Pitch-Angle Tracking
% Cessna 182 Longitudinal Model
%
% States: x = [u alpha q theta]^T
% Augmented integrator state:
%   xi = integral(theta_cmd - theta)
%
% Control law:
%   delta_e = -Kx*x - Ki*xi
%
% This avoids relying on Nbar and usually gives much better
% tracking behavior than plain LQR + prefilter for this model.
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
Ctheta = [0 0 0 1];

x0 = zeros(4,1);
xi0 = 0;

%% ------------------------------------------------------------
% 2) Augment with integral of theta error
%
% x_aug = [x; xi]
% xi_dot = theta_cmd - theta = r - Ctheta*x
%
% So:
% [x_dot ] = [ A      0 ][x ] + [B] delta_e + [0] r
% [xi_dot]   [-Cth    0 ][xi]   [0]          [1]
% ------------------------------------------------------------
A_aug = [A            zeros(4,1);
        -Ctheta       0        ];

B_aug = [B;
         0];

E_aug = [zeros(4,1);
         1];

%% ------------------------------------------------------------
% 3) LQI weights
%
% Start conservative.
% Increase Qxi if tracking is too slow.
% Increase R if elevator demand is too aggressive.
% Increase q-weight if pitch-rate overshoot is too high.
% ------------------------------------------------------------
Qx  = diag([0.01, 10, 25, 8]);
Qxi = 80;

Q_aug = blkdiag(Qx, Qxi);
R = 15;

K_aug = lqr(A_aug, B_aug, Q_aug, R);

Kx = K_aug(1:4);
Ki = K_aug(5);

Acl = [A - B*Kx,    -B*Ki;
       -Ctheta,      0    ];

Bcl = [zeros(4,1);
       1];

Ccl = [eye(4), zeros(4,1)];
Dcl = zeros(4,1);

sys_lqi = ss(Acl, Bcl, Ccl, Dcl);

%% ------------------------------------------------------------
% 4) Command input
% ------------------------------------------------------------
theta_cmd_deg = 5;
theta_cmd = deg2rad(theta_cmd_deg);

t = linspace(0,80,6000);
r = theta_cmd * ones(size(t));

x0_aug = [x0; xi0];

%% ------------------------------------------------------------
% 5) Simulate
% ------------------------------------------------------------
[y,t,x_aug_resp] = lsim(sys_lqi, r, t, x0_aug);

x_resp = x_aug_resp(:,1:4);
xi_resp = x_aug_resp(:,5);

%% ------------------------------------------------------------
% 6) Extract states
% ------------------------------------------------------------
u_resp     = x_resp(:,1);              % ft/s
alpha_resp = rad2deg(x_resp(:,2));     % deg
q_resp     = rad2deg(x_resp(:,3));     % deg/s
theta_resp = rad2deg(x_resp(:,4));     % deg

theta_cmd_plot = theta_cmd_deg * ones(size(t));

%% ------------------------------------------------------------
% 7) Elevator demand
% delta_e = -Kx*x - Ki*xi
% ------------------------------------------------------------
delta_e = -(x_resp * Kx.') - Ki*xi_resp;   % rad
delta_e_deg = rad2deg(delta_e);

%% ------------------------------------------------------------
% 8) Poles and gains
% ------------------------------------------------------------
poles_lqi = eig(Acl);

disp('--------------------------------------------');
disp('Phase 3C2: LQI / Servo-LQR');
disp('Kx = ');
disp(Kx);
disp(['Ki = ', num2str(Ki)]);
disp('Closed-loop poles:');
disp(poles_lqi);
disp(['Final theta (deg) = ', num2str(theta_resp(end))]);
disp(['Theta command (deg) = ', num2str(theta_cmd_deg)]);

%% ------------------------------------------------------------
% 9) Theta tracking
% ------------------------------------------------------------
figure('Color','w');
plot(t, theta_resp, 'LineWidth', 1.8); hold on;
plot(t, theta_cmd_plot, '--', 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('\theta (deg)');
title('Phase 3C2: LQI Pitch-Angle Tracking');
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
title('Phase 3C2: LQI State Responses');

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
title('Phase 3C2: LQI Elevator Demand');

%% ------------------------------------------------------------
% 12) Pole map
% ------------------------------------------------------------
figure('Color','w');
plot(real(eig(A)), imag(eig(A)), 'x', 'MarkerSize', 10, 'LineWidth', 2); hold on;
plot(real(poles_lqi), imag(poles_lqi), 'o', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
xlabel('Real Axis');
ylabel('Imaginary Axis');
title('Phase 3C2: Open-Loop vs LQI Closed-Loop Poles');
legend('Open-loop','LQI','Location','best');