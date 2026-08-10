function alt = augment_altitude_model(model, cfg)
%AUGMENT_ALTITUDE_MODEL Add small-perturbation altitude kinematics.
%
% Original Phase 1 state:
%   x = [u w q theta]^T
%
% Under the documented small-angle, approximately level-flight convention:
%   h_dot ~= U0*theta - w
%
% Augmented state:
%   x_h = [u w q theta h]^T

if model.n ~= 4
    error(['Altitude augmentation is currently implemented for the four-state ' ...
           'Phase 1 model x=[u w q theta]^T only.']);
end

if ~isfinite(model.U0)
    error(['Trim airspeed U0 is missing. Re-export the Phase 1 model with ' ...
           'import_phase1_model(...,''U0'',U0).']);
end

Hh = zeros(1,4);
Hh(cfg.idx.w) = -1;
Hh(cfg.idx.theta) = model.U0;

Aaug = [model.A, zeros(4,1);
        Hh,       0];

Baug = [model.Be;
        0];

alt.A = Aaug;
alt.B = Baug;
alt.Hh = Hh;
alt.state_names = [model.state_names, {'h'}];
alt.n = 5;
alt.U0 = model.U0;

end
