function load_params(modelIdOverride)
    % try to load previous parameters
    try
        clear params modelId;
        load("system_parameters/params.mat");
        paramsPrev = params;
        modelIdPrev = modelId;
    catch
        paramsPrev = struct([]);
        modelIdPrev = -1;
    end
    
    % wanted params
    parameters_maggy_V41;
    
    % wanted model (use override if provided)
    if nargin >= 1
        modelId = modelIdOverride;
    else
        modelId = MaglevModel.Accurate;
    end
    
    if (~isequal(paramsPrev, params) || ~isequal(modelIdPrev, modelId)) && ~isequal(modelId, MaglevModel.Accurate)
        fprintf("recalculating params\n");
        correctionFactor = computeSolenoidRadiusCorrectionFactor(params, modelId);
        paramsFixed = params;
        paramsFixed.solenoids.r = correctionFactor * paramsFixed.solenoids.r;
        save("system_parameters/params.mat", "params", "paramsFixed", "modelId");
    end
    
    % export to base workspace
    assignin('base', 'params', paramsFixed);
    fprintf("Params loaded!\n");
end