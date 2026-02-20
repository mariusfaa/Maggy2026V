function [K, E] = ellipke_approx(m)
%ELLIPKE_APPROX Approximation of complete elliptic integrals
%   [K, E] = ellipke_approx(m)
%   Input:
%       m - parameter (m = k^2)
%   Output:
%       K - approximate complete elliptic integral of the first kind
%       E - approximate complete elliptic integral of the second kind
%
%   Uses:
%       E_appr(k) = π/2 - 0.567 * k^(2.4 + (k+0.1)^5.8)
%       K_appr(k) = π / (2*(1-k)^0.19) - 0.17*(k + 0.015)^0.8

    % Convert from m to k
    k = sqrt(m);

    % Ensure valid domain (real values for 0 <= m <= 1)
    % if any(k < 0 | k > 1)
    %     error('Input m must satisfy 0 <= m <= 1.');
    % end

    % Approximation for E(k)
    E = pi/2 - 0.567 .* k.^(2.4 + (k + 0.1).^5.8);

    % Approximation for K(k)
    K = pi ./ (2 .* (1 - k).^0.19) - 0.17 .* (k + 0.015).^0.8;

end