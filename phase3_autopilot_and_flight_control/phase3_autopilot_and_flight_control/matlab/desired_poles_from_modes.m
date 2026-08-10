function poles = desired_poles_from_modes(wn_sp, zeta_sp, wn_ph, zeta_ph)
%DESIRED_POLES_FROM_MODES Construct two desired complex pole pairs.
%
% Inputs:
%   wn_sp, zeta_sp - short-period natural frequency and damping ratio
%   wn_ph, zeta_ph - phugoid natural frequency and damping ratio

validateattributes(wn_sp, {'numeric'}, {'scalar','positive','finite'});
validateattributes(wn_ph, {'numeric'}, {'scalar','positive','finite'});
validateattributes(zeta_sp, {'numeric'}, {'scalar','>',0,'<',1,'finite'});
validateattributes(zeta_ph, {'numeric'}, {'scalar','>',0,'<',1,'finite'});

sigma_sp = -zeta_sp * wn_sp;
wd_sp = wn_sp * sqrt(1 - zeta_sp^2);

sigma_ph = -zeta_ph * wn_ph;
wd_ph = wn_ph * sqrt(1 - zeta_ph^2);

poles = [ ...
    sigma_sp + 1i*wd_sp
    sigma_sp - 1i*wd_sp
    sigma_ph + 1i*wd_ph
    sigma_ph - 1i*wd_ph ];

end
