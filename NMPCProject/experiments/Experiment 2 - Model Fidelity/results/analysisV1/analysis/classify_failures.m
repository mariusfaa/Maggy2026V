function T = classify_failures(T)
% CLASSIFY_FAILURES  Add a categorical failure column derived from error_message.
%
%   ok                  completed == 1
%   no_equilibrium      computeSolenoidRadiusCorrectionFactor / no real equilibrium
%   compiler_killed     cc1 / Killed signal during compile_ocp_shared_lib
%   other_failure       anything else with completed == 0

    n = height(T);
    cat = strings(n, 1);
    msg = string(T.error_message);
    completed = double(T.completed);

    for i = 1:n
        if completed(i) == 1
            cat(i) = "ok";
        else
            m = msg(i);
            if contains(m, "computeSolenoidRadiusCorrectionFactor", 'IgnoreCase', true) ...
                    || contains(m, "No real equilibrium", 'IgnoreCase', true)
                cat(i) = "no_equilibrium";
            elseif contains(m, "Killed signal", 'IgnoreCase', true) ...
                    || contains(m, "compile_ocp_shared_lib", 'IgnoreCase', true) ...
                    || contains(m, "cc1", 'IgnoreCase', true)
                cat(i) = "compiler_killed";
            else
                cat(i) = "other_failure";
            end
        end
    end

    T.failure_category = categorical(cat, ...
        {'ok','no_equilibrium','compiler_killed','other_failure'});
end
