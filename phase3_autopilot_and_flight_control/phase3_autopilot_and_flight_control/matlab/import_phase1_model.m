function save_path = import_phase1_model(A, B, varargin)
%IMPORT_PHASE1_MODEL Export the actual Phase 1 longitudinal model for Phase 3.
%
% This function is the preferred bridge from the existing Phase 1/2 workspace
% into Phase 3.
%
% Required:
%   A - longitudinal state matrix
%   B - control-input matrix
%
% Name-value options:
%   'U0'          trim airspeed [m/s]
%   'h0'          trim altitude [m]
%   'Kq'          Phase 2 pitch-rate-damper gain
%   'C'           output matrix
%   'D'           feedthrough matrix
%   'StateNames'  cell array of state names
%   'InputNames'  cell array of input names
%   'SavePath'    destination .mat file
%
% Example, run from the Phase 1/2 MATLAB workspace:
%
%   import_phase1_model(A, B, ...
%       'U0', U0, ...
%       'h0', h0, ...
%       'Kq', Kq);
%
% The default saved file is:
%   ../data/phase1_longitudinal_model.mat

validateattributes(A, {'numeric'}, {'2d','square','finite'});
validateattributes(B, {'numeric'}, {'2d','finite','nrows',size(A,1)});

n = size(A,1);
m = size(B,2);

p = inputParser;
p.addParameter('U0', NaN, @(x) isnumeric(x) && isscalar(x));
p.addParameter('h0', NaN, @(x) isnumeric(x) && isscalar(x));
p.addParameter('Kq', NaN, @(x) isnumeric(x) && isscalar(x));
p.addParameter('C', eye(n), @(x) isnumeric(x) && size(x,2) == n);
p.addParameter('D', zeros(n,m), @(x) isnumeric(x));
p.addParameter('StateNames', {'u','w','q','theta'}, @(x) iscell(x));
p.addParameter('InputNames', cellstr("input_" + (1:m)), @(x) iscell(x));
p.addParameter('SavePath', '', @(x) ischar(x) || isstring(x));
p.parse(varargin{:});

U0 = p.Results.U0;
h0 = p.Results.h0;
Kq = p.Results.Kq;
C  = p.Results.C;
D  = p.Results.D;
state_names = p.Results.StateNames;
input_names = p.Results.InputNames;

if isempty(p.Results.SavePath)
    matlab_dir = fileparts(mfilename('fullpath'));
    project_root = fileparts(matlab_dir);
    data_dir = fullfile(project_root, 'data');
    if ~exist(data_dir, 'dir')
        mkdir(data_dir);
    end
    save_path = fullfile(data_dir, 'phase1_longitudinal_model.mat');
else
    save_path = char(p.Results.SavePath);
    save_dir = fileparts(save_path);
    if ~isempty(save_dir) && ~exist(save_dir, 'dir')
        mkdir(save_dir);
    end
end

save(save_path, 'A', 'B', 'C', 'D', 'U0', 'h0', 'Kq', ...
     'state_names', 'input_names');

fprintf('Saved Phase 1/2 model interface to:\n  %s\n', save_path);
fprintf('States: %d, Inputs: %d\n', n, m);

end
