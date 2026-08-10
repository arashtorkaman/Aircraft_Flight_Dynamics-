function baseline = validate_phase1_baseline(model, cfg)
%VALIDATE_PHASE1_BASELINE Verify the inherited Phase 1 longitudinal plant.
%
% This function checks dimensions, controllability, open-loop poles, and
% compares the current A matrix against the pole set recovered from the
% earlier Phase 1 project history. The comparison is warning-only.

A = model.A;
Be = model.Be;
n = size(A,1);

poles = eig(A);
modal = modal_metrics(poles);

Co = ctrb(A,Be);
rank_Co = rank(Co);

baseline.poles = poles;
baseline.modal = modal;
baseline.controllability_matrix = Co;
baseline.controllability_rank = rank_Co;
baseline.is_controllable = (rank_Co == n);

fprintf('\n--- PHASE 1 BASELINE VALIDATION ---\n');
fprintf('Open-loop poles:\n');
disp(poles);

fprintf('Controllability rank = %d of %d\n', rank_Co, n);

if baseline.is_controllable
    fprintf('Elevator-input model is fully controllable.\n');
else
    warning('Elevator-input model is NOT fully controllable.');
end

expected = cfg.baseline.phase1_poles(:);

if numel(expected) == numel(poles)
    unmatched = poles(:);
    max_error = 0;

    for k = 1:numel(expected)
        [err, idx] = min(abs(unmatched - expected(k)));
        max_error = max(max_error, err);
        unmatched(idx) = [];
    end

    baseline.max_expected_pole_error = max_error;

    if max_error <= cfg.validation.pole_match_tolerance
        fprintf(['Current A matrix matches the recovered Phase 1 pole set ' ...
                 'within tolerance (max error %.3g).\n'], max_error);
    else
        warning(['Current A matrix differs from the recovered Phase 1 pole set. ' ...
                 'Max nearest-pole error = %.6g. This can be legitimate if ' ...
                 'you are using a revised Phase 1 model.'], max_error);
    end
else
    baseline.max_expected_pole_error = NaN;
end

% Identify representative complex-pair modes by positive imaginary poles.
complex_pos = poles(imag(poles) > 0);

if numel(complex_pos) >= 2
    mode_data = modal_metrics(complex_pos);
    [~,order] = sort(mode_data.wn, 'descend');

    baseline.short_period_pole = complex_pos(order(1));
    baseline.phugoid_pole = complex_pos(order(end));

    fprintf('\nRepresentative short-period pole: % .6f %+.6fi\n', ...
        real(baseline.short_period_pole), imag(baseline.short_period_pole));
    fprintf('Representative phugoid pole     : % .6f %+.6fi\n', ...
        real(baseline.phugoid_pole), imag(baseline.phugoid_pole));
end

end
