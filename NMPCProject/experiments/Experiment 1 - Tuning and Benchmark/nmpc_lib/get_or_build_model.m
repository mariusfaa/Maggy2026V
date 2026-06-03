function M = get_or_build_model(model_kind, controller_n, plant_n)
% GET_OR_BUILD_MODEL  Cached wrapper around build_model.
%
%   M = get_or_build_model('reduced10', 10, 30)
%
% Cache key: (model_kind, controller_n, plant_n). Lives in a persistent
% containers.Map for the MATLAB session lifetime.

    persistent cache
    if isempty(cache)
        cache = containers.Map('KeyType','char','ValueType','any');
    end

    key = sprintf('%s_c%d_p%d', lower(model_kind), controller_n, plant_n);
    if isKey(cache, key)
        M = cache(key);
        return;
    end

    M = build_model(model_kind, controller_n, plant_n);
    cache(key) = M;
end
