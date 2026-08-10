function model = load_phase1_model(cfg)
%LOAD_PHASE1_MODEL Load the actual Phase 1 longitudinal model for Phase 3.

if nargin < 1
    cfg = phase3_config;
end

model_file = cfg.paths.model_mat;

if ~exist(model_file, 'file')
    error(['Phase 1 model file was not found:\n  %s\n\n' ...
           'Run import_phase1_model(A,B,...) from your Phase 1/2 MATLAB ' ...
           'workspace first.'], model_file);
end

S = load(model_file);

required = {'A','B'};
for k = 1:numel(required)
    if ~isfield(S, required{k})
        error('Model file is missing required variable "%s".', required{k});
    end
end

A = S.A;
B = S.B;

validateattributes(A, {'numeric'}, {'2d','square','finite'});
validateattributes(B, {'numeric'}, {'2d','finite','nrows',size(A,1)});

n = size(A,1);
m = size(B,2);

model.A = A;
model.B = B;

if isfield(S,'C')
    model.C = S.C;
else
    model.C = eye(n);
end

if isfield(S,'D')
    model.D = S.D;
else
    model.D = zeros(size(model.C,1),m);
end

if isfield(S,'U0')
    model.U0 = S.U0;
else
    model.U0 = NaN;
end

if isfield(S,'h0')
    model.h0 = S.h0;
else
    model.h0 = NaN;
end

if isfield(S,'Kq')
    model.Kq = S.Kq;
else
    model.Kq = NaN;
end

if isfield(S,'state_names')
    model.state_names = S.state_names;
else
    model.state_names = cfg.state_names;
end

if isfield(S,'input_names')
    model.input_names = S.input_names;
else
    model.input_names = cellstr("input_" + (1:m));
end

if numel(model.state_names) ~= n
    warning('State-name count does not match the number of states.');
end

if cfg.input.elevator_index > m
    error(['cfg.input.elevator_index=%d but B has only %d input column(s). ' ...
           'Correct the elevator input index in phase3_config.m.'], ...
           cfg.input.elevator_index, m);
end

model.Be = B(:,cfg.input.elevator_index);
model.elevator_index = cfg.input.elevator_index;
model.n = n;
model.m = m;

end
