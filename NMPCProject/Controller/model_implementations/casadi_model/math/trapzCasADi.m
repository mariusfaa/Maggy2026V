function integral = trapzCasADi(theta, y)
% TRAPZCASADI  Trapezoidal integration over a closed periodic domain.
%
%   integral = trapzCasADi(theta, y)
%
%   Integrates y(theta) from theta(1) to 2*pi using the trapezoidal rule,
%   closing the loop by appending y(1) at theta = 2*pi.
%   Compatible with CasADi SX/MX types.
%
% Inputs:
%   theta    - Discrete angles (1xN numeric array, not necessarily uniform)
%   y        - Function values  (1xN CasADi SX/MX)
%
% Output:
%   integral - Scalar integral value (CasADi SX/MX)

    y_closed    = [y, y(1)];
    dtheta_vec  = diff([theta, 2*pi]);
    integral    = sum(0.5 * (y_closed(1:end-1) + y_closed(2:end)) .* dtheta_vec);
end
