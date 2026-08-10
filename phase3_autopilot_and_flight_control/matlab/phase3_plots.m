function phase3_plots(model, baseline, sf, lqr_ctl, sims, comparison, cfg)
%PHASE3_PLOTS Generate and optionally save Phase 3 portfolio figures.

if cfg.output.close_existing_figures
    close all;
end

if ~exist(cfg.paths.figures_dir,'dir')
    mkdir(cfg.paths.figures_dir);
end

%% 1. Pole comparison
f1 = figure('Name','Phase 3 Pole Comparison');
hold on;
grid on;

plot(real(baseline.poles),imag(baseline.poles),'x', ...
    'MarkerSize',10,'LineWidth',1.5);

labels = {'Open loop'};

if sf.available
    plot(real(sf.poles),imag(sf.poles),'o', ...
        'MarkerSize',8,'LineWidth',1.5);
    labels{end+1} = 'State feedback';
end

plot(real(lqr_ctl.poles),imag(lqr_ctl.poles),'s', ...
    'MarkerSize',8,'LineWidth',1.5);
labels{end+1} = 'LQR';

xlabel('Real axis [1/s]');
ylabel('Imaginary axis [rad/s]');
title('Phase 3 Open-Loop and Closed-Loop Poles');
legend(labels,'Location','best');

save_if_requested(f1,'closed_loop_poles.png',cfg);

%% 2. Pitch response
f2 = figure('Name','Pitch Command Tracking');
hold on;
grid on;

plot(sims.pitch_lqr.t,rad2deg(sims.pitch_lqr.theta),'LineWidth',1.5);
plot(sims.pitch_lqr.t, ...
    rad2deg(sims.pitch_lqr.theta_cmd)*ones(size(sims.pitch_lqr.t)), ...
    '--','LineWidth',1.2);

labels = {'LQR','Command'};

if isfield(sims,'pitch_state_feedback') && ...
        sims.pitch_state_feedback.available
    plot(sims.pitch_state_feedback.t, ...
        rad2deg(sims.pitch_state_feedback.theta),'LineWidth',1.2);
    labels{end+1} = 'State feedback';
end

xlabel('Time [s]');
ylabel('Pitch angle \theta [deg]');
title('Pitch-Angle Command Tracking');
legend(labels,'Location','best');

save_if_requested(f2,'pitch_response.png',cfg);

%% 3. Pitch-rate response
f3 = figure('Name','Pitch Rate');
hold on;
grid on;

plot(sims.pitch_lqr.t,rad2deg(sims.pitch_lqr.q),'LineWidth',1.5);

if isfield(sims,'pitch_state_feedback') && ...
        sims.pitch_state_feedback.available
    plot(sims.pitch_state_feedback.t, ...
        rad2deg(sims.pitch_state_feedback.q),'LineWidth',1.2);
    legend('LQR','State feedback','Location','best');
else
    legend('LQR','Location','best');
end

xlabel('Time [s]');
ylabel('Pitch rate q [deg/s]');
title('Pitch-Rate Response');

save_if_requested(f3,'pitch_rate_response.png',cfg);

%% 4. Elevator command
f4 = figure('Name','Elevator Command');
hold on;
grid on;

plot(sims.pitch_lqr.t,rad2deg(sims.pitch_lqr.delta),'LineWidth',1.5);

if isfield(sims,'pitch_state_feedback') && ...
        sims.pitch_state_feedback.available
    plot(sims.pitch_state_feedback.t, ...
        rad2deg(sims.pitch_state_feedback.delta),'LineWidth',1.2);
    legend('LQR','State feedback','Location','best');
else
    legend('LQR','Location','best');
end

xlabel('Time [s]');
ylabel('Elevator \delta_e [deg]');
title('Elevator Control Effort');

save_if_requested(f4,'elevator_command.png',cfg);

%% 5. Altitude response, only if available
if sims.altitude_lqr.available
    f5 = figure('Name','Altitude Response');
    hold on;
    grid on;

    plot(sims.altitude_lqr.t,sims.altitude_lqr.h,'LineWidth',1.5);
    plot(sims.altitude_lqr.t, ...
        sims.altitude_lqr.h_cmd*ones(size(sims.altitude_lqr.t)), ...
        '--','LineWidth',1.2);

    xlabel('Time [s]');
    ylabel('Altitude perturbation h [m]');
    title('LQR Autopilot Altitude Tracking');
    legend('Altitude','Command','Location','best');

    save_if_requested(f5,'altitude_response.png',cfg);

    f6 = figure('Name','Altitude Autopilot Pitch Command');
    hold on;
    grid on;

    plot(sims.altitude_lqr.t, ...
        rad2deg(sims.altitude_lqr.theta_cmd),'LineWidth',1.5);
    plot(sims.altitude_lqr.t, ...
        rad2deg(sims.altitude_lqr.theta),'LineWidth',1.2);

    xlabel('Time [s]');
    ylabel('Pitch angle [deg]');
    title('Altitude-Loop Pitch Command and Aircraft Pitch');
    legend('\theta_c','\theta','Location','best');

    save_if_requested(f6,'altitude_pitch_command.png',cfg);
end

%% 6. Controller summary table to command window
fprintf('\nController summary used for plots:\n');
disp(comparison.summary);

end

function save_if_requested(fig,filename,cfg)

if cfg.output.save_figures
    exportgraphics(fig,fullfile(cfg.paths.figures_dir,filename), ...
        'Resolution',180);
end

end
