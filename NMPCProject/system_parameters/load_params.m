% try to load previous parameters to skip some tedious calculations
try
    load("system_parameters/params.mat");
    paramsPrev = params;
catch
    paramsPrev = struct([]);
end

% wanted params
parameters_maggy_V41;

% wanted model
modelId = MaglevModel.Fast;

if ~isequal(paramsPrev, params)
    fprintf("recalculating params\n");
    correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params,modelId);
    paramsFixed = params;
    paramsFixed.solenoids.r = correctionFactorFast*paramsFixed.solenoids.r;
    save("system_parameters/params.mat", "params", "paramsFixed");
end

% use fixed version
params = paramsFixed;

clear paramsPrev;

fprintf("Params loaded!\n");