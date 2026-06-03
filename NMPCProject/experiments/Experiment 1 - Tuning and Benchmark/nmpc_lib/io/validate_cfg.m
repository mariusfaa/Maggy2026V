function cfg_out = validate_cfg(cfg_in, varargin)
% VALIDATE_CFG  Validate a cfg struct against cfg_schema() and fill defaults.
%
%   cfg_out = validate_cfg(cfg_in)
%   cfg_out = validate_cfg(cfg_in, 'PrintDefaults', false)
%
% Behaviour:
%   - Required fields missing  -> error with a clear message.
%   - Required fields with wrong type / failing validator -> error.
%   - Optional fields missing  -> default applied and (by default) printed.
%   - Unknown fields present   -> warning (not error), to allow forward-compat.
%   - plant_override.*         -> recursively validated; the matched-plant
%                                 defaults are filled here so build_plant
%                                 sees a fully-resolved cfg.
%
% This function is intentionally strict on required fields and lenient on
% extras. Every stage runner should call it on every cfg before doing work.

    p = inputParser();
    p.addRequired('cfg_in', @isstruct);
    p.addParameter('PrintDefaults', true, @islogical);
    p.parse(cfg_in, varargin{:});
    print_defaults = p.Results.PrintDefaults;

    S       = cfg_schema();
    cfg_out = struct();

    % Walk the schema and fill cfg_out.
    schema_fields = fieldnames(S);
    for i = 1:numel(schema_fields)
        fn   = schema_fields{i};
        spec = S.(fn);
        if is_field_spec(spec)
            cfg_out.(fn) = resolve_field(cfg_in, fn, spec, print_defaults);
        else
            % Nested struct (e.g., plant_override) — recurse.
            sub_in  = struct_or_empty(cfg_in, fn);
            sub_out = struct();
            sub_schema_fields = fieldnames(spec);
            for j = 1:numel(sub_schema_fields)
                sn   = sub_schema_fields{j};
                ssub = spec.(sn);
                sub_out.(sn) = resolve_field(sub_in, [fn '.' sn], ssub, print_defaults);
            end
            cfg_out.(fn) = sub_out;
        end
    end

    % Warn about unknown top-level fields.
    extras = setdiff(fieldnames(cfg_in), schema_fields);
    for k = 1:numel(extras)
        warning('validate_cfg:unknown_field', ...
            'cfg has unknown field "%s" (ignored)', extras{k});
    end

    % --- Cross-field consistency rules -----------------------------------

    % SQP_RTI implies nlp_solver_max_iter = 1.
    if strcmp(cfg_out.nlp_solver_type, 'SQP_RTI')
        cfg_out.nlp_solver_max_iter = 1;
    end

    % If any plant_override.* differs from the matched-plant default, the
    % caller must supply a non-empty .reason. Resolve "differs from default"
    % only after applying the matched-plant policy.
    po = cfg_out.plant_override;
    matched_type   = cfg_out.integrator_type;
    matched_stages = cfg_out.sim_method_num_stages;
    matched_steps  = cfg_out.sim_method_num_steps;
    is_asym = false;
    if ~isempty(po.integrator_type) && ~strcmp(po.integrator_type, matched_type);   is_asym = true; end
    if ~isempty(po.sim_method_num_stages) && po.sim_method_num_stages ~= matched_stages; is_asym = true; end
    if ~isempty(po.sim_method_num_steps)  && po.sim_method_num_steps  ~= matched_steps;  is_asym = true; end
    if is_asym && isempty(po.reason)
        error('validate_cfg:plant_override_unjustified', ...
              ['plant_override.* is asymmetric to the OCP integrator but ' ...
               'plant_override.reason is empty. Supply a justification string.']);
    end
    cfg_out.plant_override.is_asymmetric = is_asym;

    % Tf > 0 already enforced by validator; sanity check on Ts.
    Ts = cfg_out.Tf / cfg_out.N;
    if Ts <= 0
        error('validate_cfg:bad_Ts', 'Tf/N = %g is not positive', Ts);
    end
end

function v = resolve_field(cfg_in, dotted_name, spec, print_defaults)
    % cfg_in is ALREADY the parent struct that should directly contain the
    % leaf field; the dotted_name is for error messages only. (When the
    % outer walk descends into a nested struct it passes the sub-struct as
    % cfg_in, so a path-style traversal here would double-walk.)
    parts = strsplit(dotted_name, '.');
    leaf  = parts{end};

    if isstruct(cfg_in) && isfield(cfg_in, leaf)
        v = cfg_in.(leaf);
    else
        if spec.required
            error('validate_cfg:missing_required', ...
                  'Required cfg field "%s" is missing', dotted_name);
        end
        v = spec.default;
        if print_defaults
            fprintf('[cfg_default] %-40s = %s\n', dotted_name, render_value(v));
        end
    end

    % Type check.
    if ~check_type(v, spec.type)
        error('validate_cfg:bad_type', ...
              'cfg.%s has wrong type (expected %s, got %s)', ...
              dotted_name, spec.type, class(v));
    end

    % Validator.
    if ~isempty(spec.validator)
        try
            ok = spec.validator(v);
        catch ME
            error('validate_cfg:validator_error', ...
                  'cfg.%s validator threw: %s', dotted_name, ME.message);
        end
        if ~ok
            error('validate_cfg:validator_failed', ...
                  'cfg.%s failed validator', dotted_name);
        end
    end
end

function tf = is_field_spec(s)
    tf = isstruct(s) && all(isfield(s, {'type','required','default','validator','description'}));
end

function s = struct_or_empty(parent, fn)
    if isstruct(parent) && isfield(parent, fn) && isstruct(parent.(fn))
        s = parent.(fn);
    else
        s = struct();
    end
end

function tf = check_type(v, type)
    switch type
        case 'numeric'        ; tf = isnumeric(v) || isempty(v);
        case 'char'           ; tf = ischar(v);
        case 'logical'        ; tf = islogical(v);
        case 'struct'         ; tf = isstruct(v) || isempty(v);
        case 'cell'           ; tf = iscell(v);
        case 'function_handle'; tf = isa(v,'function_handle');
        otherwise             ; tf = true; % unknown type — let validator handle it
    end
end

function s = render_value(v)
    if isnumeric(v)
        if isempty(v)
            s = '[]';
        elseif isscalar(v)
            s = num2str(v);
        else
            s = sprintf('[%dx%d %s]', size(v,1), size(v,2), class(v));
        end
    elseif ischar(v)
        s = ['"' v '"'];
    elseif islogical(v)
        if v; s = 'true'; else; s = 'false'; end
    elseif isstruct(v)
        s = '<struct>';
    else
        s = ['<' class(v) '>'];
    end
end
