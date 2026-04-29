classdef Observer < handle
    % Observer  MATLAB wrapper
    %
    % Copy this file, libobserver.so, and observer_capi.h to working directory
    % Only usable on linux
    %
    % Usage
    % -----
    %   filterVariant: 0 KF
    %                  1 EKF
    %                  2 UKF (most stable)
    %
    %   obs = Observer(filterVariant, dt, xLp); % minimal
    %   obs = Observer(filterVariant, dt, xLp, Name, Value, ...);
    %
    %   stateEst = obs.run(input, meas);
    %
    %   delete(obs);   % or let it go out of scope – memory is freed
    %
    % Named options (all optional, defaults match initObserver defaults)
    % ------------------------------------------------------------------
    %  'useSRformulation'  logical, default false
    %   'RK4Iterations'     integer, default 0
    %   'updateJacobians'   logical, default true
    %   'updateQ'           logical, default false
    %   'cubature'          logical, default false
    %   'normalized'        logical, default false
    %
    % Library loading
    % ---------------
    %   The library is loaded once per MATLAB session and shared across
    %   all Observer instances.  To reload (e.g. after recompiling):
    %       Observer.unloadLib();

    % ------------------------------------------------------------------
    properties (Access = private)
        handle_   int32   % C-side handle (index into registry)
        nx_       int32   % number of states
        nu_       int32   % number of inputs
        ny_       int32   % number of measurements
    end

    % ------------------------------------------------------------------
    properties (Constant, Access = private)
        LIB_NAME  = 'observer';           % matches libobserver.so / observer.dll
        HDR_NAME  = 'observer_capi.h';    % pure-C header for loadlibrary
    end

    % ==================================================================
    methods

        function obj = Observer(filterVariant, dt, xLp, varargin)
            % Parse optional name-value pairs
            p = inputParser();
            addRequired (p, 'filterVariant', @(x) isnumeric(x) && isscalar(x));
            addRequired (p, 'dt',            @(x) isnumeric(x) && isscalar(x));
            addRequired (p, 'xLp',           @(x) isnumeric(x) && isvector(x));
            addParameter(p, 'useSRformulation', false, @islogical);
            addParameter(p, 'RK4Iterations',    0,     @(x) isnumeric(x) && isscalar(x));
            addParameter(p, 'updateJacobians',  true,  @islogical);
            addParameter(p, 'updateQ',          false, @islogical);
            addParameter(p, 'cubature',         false, @islogical);
            addParameter(p, 'normalized',       false, @islogical);
            parse(p, filterVariant, dt, xLp, varargin{:});
            r = p.Results;

            Observer.loadLib();

            xLp_col = double(r.xLp(:));   % ensure column, double
            nx      = int32(numel(xLp_col));

            % Wrap xLp in a libpointer so calllib passes a double*
            xLp_ptr = libpointer('doublePtr', xLp_col);

            h = calllib(Observer.LIB_NAME, 'observer_init', ...
                int32(r.filterVariant), ...
                double(r.dt),           ...
                xLp_ptr,                ...
                nx,                     ...
                int32(r.useSRformulation), ...
                int32(r.RK4Iterations),    ...
                int32(r.updateJacobians),  ...
                int32(r.updateQ),          ...
                int32(r.cubature),        ...
                int32(r.normalized));

            if h < 0
                error('Observer:initFailed', ...
                      'observer_init returned invalid handle (%d).', h);
            end

            obj.handle_ = int32(h);
            obj.nx_     = int32(calllib(Observer.LIB_NAME, 'observer_get_nx', h));
            obj.nu_     = int32(calllib(Observer.LIB_NAME, 'observer_get_nu', h));
            obj.ny_     = int32(calllib(Observer.LIB_NAME, 'observer_get_ny', h));
        end

        % --------------------------------------------------------------
        function stateEst = run(obj, input, meas)
            % RUN  Execute one filter update step.
            %
            %   stateEst = obs.run(input, meas)
            %
            %   input    (nu x 1) double – control input vector
            %   meas     (ny x 1) double – measurement vector
            %   stateEst (nx x 1) double – updated state estimate

            input = double(input(:));
            meas  = double(meas(:));

            nu_in = int32(numel(input));
            ny_in = int32(numel(meas));

            if nu_in ~= obj.nu_
                error('Observer:dimMismatch', ...
                      'input length %d does not match expected %d.', nu_in, obj.nu_);
            end
            if ny_in ~= obj.ny_
                error('Observer:dimMismatch', ...
                      'meas length %d does not match expected %d.', ny_in, obj.ny_);
            end

            % Pre-allocate output buffer
            states_ptr = libpointer('doublePtr', zeros(obj.nx_, 1));

            ret = calllib(Observer.LIB_NAME, 'observer_run', ...
                obj.handle_,                          ...
                libpointer('doublePtr', input), nu_in, ...
                libpointer('doublePtr', meas),  ny_in, ...
                states_ptr, obj.nx_);

            if ret ~= 0
                error('Observer:runFailed', ...
                      'observer_run returned error code %d.', ret);
            end

            stateEst = states_ptr.Value;   % (nx x 1) double
        end

        % --------------------------------------------------------------
        function delete(obj)
            % Destructor – frees the C++ KalmanFilter object.
            if ~isempty(obj.handle_) && obj.handle_ >= 0
                if libisloaded(Observer.LIB_NAME)
                    calllib(Observer.LIB_NAME, 'observer_destroy', obj.handle_);
                end
                obj.handle_ = int32(-1);
            end
        end

        % --------------------------------------------------------------
        % Read-only accessors
        function n = nx(obj), n = double(obj.nx_); end
        function n = nu(obj), n = double(obj.nu_); end
        function n = ny(obj), n = double(obj.ny_); end

    end   % public methods

    % ==================================================================
    methods (Static)

        function loadLib()
            % Load the shared library if not already loaded.
            if libisloaded(Observer.LIB_NAME), return; end

            % Locate the library relative to this .m file
            here    = fileparts(mfilename('fullpath'));
            libfile = Observer.platformLibName();
            libpath = fullfile(here, libfile);
            hdrpath = fullfile(here, Observer.HDR_NAME);

            if ~isfile(libpath)
                error('Observer:libNotFound', ...
                      ['Shared library not found: %s\n' ...
                       'Compile observer_capi.cpp into a shared library first.'], libpath);
            end

            loadlibrary(libpath, hdrpath, 'alias', Observer.LIB_NAME);
        end

        function unloadLib()
            % Force-unload the library (useful after recompiling).
            if libisloaded(Observer.LIB_NAME)
                unloadlibrary(Observer.LIB_NAME);
            end
        end

    end   % static methods

    % ==================================================================
    methods (Static, Access = private)

        function name = platformLibName()
            if ispc()
                name = 'observer.dll';
            elseif ismac()
                name = 'libobserver.dylib';
            else
                name = 'libobserver.so';
            end
        end

    end

end
