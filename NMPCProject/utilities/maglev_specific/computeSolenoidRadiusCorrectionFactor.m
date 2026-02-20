function c = computeSolenoidRadiusCorrectionFactor(params,modelId)
    % Find equilibrium (used in objective function)
    zEq = computeSystemEquilibria(params,modelId);

    try
        zEq = zEq(1);
    catch
        disp('No real equilibrium for this system')
        return;
    end

    % Optimize
    options = optimoptions('fmincon', 'Display', 'off');
    c = fmincon(@(c) objfun(c,zEq,params,modelId),1,[],[],[],[],0,inf,[],options);

    % Helping functions
    function fz = computeVerticalForce(z,params,modelId)
        u = 100*ones(length(params.solenoids.r),1); % Non-zero u in order to test solenoids
        
        % Compute normal graf
        fz = zeros(size(z));
        for j = 1:length(z)
            x = [0;0;z(j);zeros(9,1)];
            [~,~,fz] = computeForceAndTorque(x,u,params,modelId);
        end
        fz = fz - 9.81*params.magnet.m;
    end

    function J = objfun(c,zEq,params,modelId)
        paramsCorrected = params;
        paramsCorrected.solenoids.r = c*paramsCorrected.solenoids.r;

        J = norm(computeVerticalForce(zEq,params,MaglevModel.Filament) - computeVerticalForce(zEq,paramsCorrected,modelId));
    end

end