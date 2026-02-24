function load_params(modelIdOverride)
    % Load previously cached parameters if available
    try
        cached = load("system_parameters/params.mat");
        paramsPrev = cached.params;
        modelIdPrev = cached.modelId;
    catch
        paramsPrev = struct([]);
        modelIdPrev = -1;
    end

    % Load current parameters from definition script
    parameters_maggy_V41;
    assert(exist('params', 'var'), 'parameters_maggy_V41 did not define ''params''');

    % Set model ID
    if nargin >= 1 && ~isempty(modelIdOverride)
        modelId = modelIdOverride;
    else
        modelId = MaglevModel.Accurate;
    end

    % Recompute only if params or modelId changed, and model is not Accurate
    paramsChanged = ~isequal(paramsPrev, params);
    modelChanged  = ~isequal(modelIdPrev, modelId);
    needsRecompute = (paramsChanged || modelChanged) && ~isequal(modelId, MaglevModel.Accurate);

    if needsRecompute
        fprintf('Recalculating params (model=%s)...\n', char(modelId));
        paramsFixed = params;
        % paramsFixed.solenoids.r = paramsFixed.solenoids.r / computeSolenoidRadiusCorrectionFactor(params, modelId);
        save("system_parameters/params.mat", "params", "paramsFixed", "modelId");
        assignin('base', 'params', paramsFixed);
    else
        assignin('base', 'params', params);
    end

    assignin('base', 'modelId', modelId);
    fprintf('Params loaded (model=%s)\n', char(modelId));
end