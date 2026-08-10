%% RUN_PHASE3
% Top-level Phase 3 workflow:
%
%   Phase 1 plant
%       -> validation
%       -> Phase 3 pole-placement state feedback (if targets supplied)
%       -> LQR synthesis
%       -> pitch command tracking
%       -> altitude-hold LQR autopilot (when U0 and PI gains are supplied)
%       -> disturbance rejection
%       -> plots
%       -> generated result file
%
% FIRST-TIME SETUP
% ----------------
% From the actual Phase 1/2 workspace, export your aircraft model:
%
%   import_phase1_model(A,B,'U0',U0,'h0',h0,'Kq',Kq);
%
% If U0, h0, or Kq are not yet available, omit those name-value pairs.
%
% Then run this script.

clearvars -except A B C D U0 h0 Kq
clc;

this_file = mfilename('fullpath');
matlab_dir = fileparts(this_file);
project_root = fileparts(matlab_dir);

addpath(matlab_dir);

cfg = phase3_config(project_root);

fprintf('============================================================\n');
fprintf('PHASE 3 — AUTOPILOT AND FLIGHT CONTROL\n');
fprintf('============================================================\n');

%% 1. Load Phase 1 aircraft model
model = aircraft_model(cfg);

%% 2. Validate inherited baseline
baseline = validate_phase1_baseline(model,cfg);

%% 3. Full-state pole-placement design
sf = state_feedback_design(model,cfg);

%% 4. LQR design
lqr_ctl = lqr_design(model,cfg);

%% 5. Simulations
sims = autopilot_simulation(model,sf,lqr_ctl,cfg);

%% 6. Controller comparison
comparison = controller_comparison(model,baseline,sf,lqr_ctl);

%% 7. Numerical result extraction
results = phase3_results_calculator( ...
    model,baseline,sf,lqr_ctl,sims,comparison,cfg);

%% 8. Figures
phase3_plots(model,baseline,sf,lqr_ctl,sims,comparison,cfg);

%% 9. Export results
export_phase3_results(results,cfg);

fprintf('\n============================================================\n');
fprintf('PHASE 3 RUN COMPLETE\n');
fprintf('============================================================\n');

if ~sf.available
    fprintf(['State-feedback pole placement was skipped because no desired ' ...
             'pole targets were configured.\n']);
end

if ~sims.altitude_lqr.available
    fprintf(['Altitude-hold simulation is not yet active. Restore U0 and ' ...
             'set cfg.altitude.Kp/Ki after tuning.\n']);
end
