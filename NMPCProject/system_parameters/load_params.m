% try to load previous parameters to skip some tedious calculations
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

% wanted model
modelId = MaglevModel.Accurate;

if ~isequal(paramsPrev, params) || ~isequal(modelIdPrev, modelId)
    fprintf("recalculating params\n");
    correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params,modelId);
    paramsFixed = params;
    paramsFixed.solenoids.r = correctionFactorFast*paramsFixed.solenoids.r;
    save("system_parameters/params.mat", "params", "paramsFixed", "modelId");
end

% use fixed version
params = paramsFixed;

clear paramsPrev modelIdPrev paramsFixed;

fprintf("Params loaded!\n");