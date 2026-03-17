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

    % Apply solenoid radius correction for Fast/Filament models
    if ~isequal(modelId, MaglevModel.Accurate)
        fprintf('Applying solenoid radius correction (model=%s)...\n', char(modelId));
        params.solenoids.r = params.solenoids.r * computeSolenoidRadiusCorrectionFactor(params, modelId);
    end

    % Only assign to base workspace when called without output (e.g. from simSetup)
    if nargout == 0
        assignin('base', 'params', params);
        assignin('base', 'modelId', modelId);
    end

    fprintf('Params loaded (model=%s)\n', char(modelId));
end
