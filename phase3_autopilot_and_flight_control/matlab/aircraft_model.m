function model = aircraft_model(cfg)
%AIRCRAFT_MODEL Phase 3 interface to the Phase 1 longitudinal plant.
%
% Phase 3 intentionally reuses the Phase 1 aircraft plant instead of creating
% a different A/B model.
%
% Usage:
%   cfg = phase3_config;
%   model = aircraft_model(cfg);

if nargin < 1
    cfg = phase3_config;
end

model = load_phase1_model(cfg);

fprintf('\n--- PHASE 3 AIRCRAFT MODEL ---\n');
fprintf('States : %d\n', model.n);
fprintf('Inputs : %d\n', model.m);
fprintf('U0     : %.8g m/s\n', model.U0);
fprintf('h0     : %.8g m\n', model.h0);

fprintf('\nA =\n');
disp(model.A);

fprintf('B =\n');
disp(model.B);

end
