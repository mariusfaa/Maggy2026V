function params = load_params(modelIdOverride)
% LOAD_PARAMS  Load and configure maglev system parameters.
%
%   params = load_params(modelId)   — returns params, no side effects
%   load_params(modelId)            — assigns params & modelId to base workspace

    % Load current parameters from definition script
    parameters_maggy_V41;

    % Set model ID
    if nargin >= 1 && ~isempty(modelIdOverride)
        modelId = modelIdOverride;
    else
        modelId = MaglevModel.Accurate;
    end

    if isequal(modelId, MaglevModel.Accurate)
        params.magnet.n = 80;
        params.magnet.n_axial = 7;
    end

    % Apply solenoid radius correction for Fast/Filament models
    % (not needed for Accurate or Dipole)
    if ~isequal(modelId, MaglevModel.Accurate) && ~isequal(modelId, MaglevModel.Dipole)
        fprintf('Applying solenoid radius correction (model=%s)...\n', char(modelId));
        params.solenoids.r = params.solenoids.r * computeSolenoidRadiusCorrectionFactor(params, modelId);
    end

    % Apply permanent magnet + solenoid correction for Dipole model
    if isequal(modelId, MaglevModel.Dipole)
        fprintf('Applying dipole correction...\n');
        [c_perm, c_sol] = computeDipoleCorrectionFactor(params);
        params.permanent.J = c_perm * params.permanent.J;
        params.solenoids.nw = c_sol * params.solenoids.nw;

        % Compute gradient correction to match Accurate model's Jacobian
        fprintf('Computing gradient correction...\n');
        [zEq_dip, ~, ~, ~] = computeSystemEquilibria(params, MaglevModel.Dipole);
        [K, B_corr] = computeDipoleGradientCorrection(params, zEq_dip(1));
        params.dipole_corr.K      = K;       % 5x5 state Jacobian correction
        params.dipole_corr.B_corr = B_corr;  % 5x4 input Jacobian correction
        params.dipole_corr.zEq    = zEq_dip(1);
    end

    % Only assign to base workspace when called without output (e.g. from simSetup)
    if nargout == 0
        assignin('base', 'params', params);
        assignin('base', 'modelId', modelId);
    end

    fprintf('Params loaded (model=%s)\n', char(modelId));
end
