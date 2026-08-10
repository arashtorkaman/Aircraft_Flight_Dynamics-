function modal = modal_metrics(poles)
%MODAL_METRICS Calculate natural frequency and damping ratio for poles.

poles = poles(:);
wn = abs(poles);

zeta = NaN(size(poles));
nonzero = wn > eps;
zeta(nonzero) = -real(poles(nonzero)) ./ wn(nonzero);

wd = abs(imag(poles));

modal.poles = poles;
modal.wn = wn;
modal.zeta = zeta;
modal.wd = wd;

modal.table = table(poles, wn, zeta, wd, ...
    'VariableNames', {'Pole','NaturalFrequency_rad_s','DampingRatio', ...
                      'DampedFrequency_rad_s'});

end
