function Nbar = reference_prefilter(A, B, K, Cref)
%REFERENCE_PREFILTER SISO reference prefilter for state feedback.
%
% Controller:
%   u = -Kx + Nbar*r
%
% Closed-loop:
%   xdot = (A-BK)x + B*Nbar*r
%
% For a constant reference and y=Cref*x, choose:
%   Nbar = -1 / ( Cref * (A-BK)^(-1) * B )

Acl = A - B*K;

denom = Cref * (Acl \ B);

if ~isscalar(denom)
    error('reference_prefilter currently supports SISO reference tracking.');
end

if abs(denom) < 1e-10
    error(['Reference prefilter is singular or ill-conditioned for the ' ...
           'selected output. Use integral augmentation or another tracking ' ...
           'architecture.']);
end

Nbar = -1 / denom;

end
