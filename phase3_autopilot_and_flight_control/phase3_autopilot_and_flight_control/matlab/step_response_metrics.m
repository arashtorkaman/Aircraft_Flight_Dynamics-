function metrics = step_response_metrics(y, t, target)
%STEP_RESPONSE_METRICS Compute common response metrics for recorded data.

y = y(:);
t = t(:);

metrics.target = target;
metrics.final_value = y(end);
metrics.steady_state_error = target - y(end);
metrics.peak_absolute = max(abs(y));

try
    info = stepinfo(y,t,target);
    metrics.rise_time = info.RiseTime;
    metrics.settling_time = info.SettlingTime;
    metrics.overshoot_percent = info.Overshoot;
    metrics.peak = info.Peak;
    metrics.peak_time = info.PeakTime;
catch
    metrics.rise_time = NaN;
    metrics.settling_time = NaN;
    metrics.overshoot_percent = NaN;
    metrics.peak = NaN;
    metrics.peak_time = NaN;
end

end
