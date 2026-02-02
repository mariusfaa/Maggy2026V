%% --- PROJECT SETUP ---
clear all; clc;

% 1. DEFINE PATHS (Double check these once more)
acados_root = 'C:\Users\mariujf\acados'; 
project_root = 'C:\Users\mariujf\NMPCProject'; 

% 2. THE NUCLEAR FIX FOR PATHS
% We must set these BEFORE importing casadi or creating the solver.
setenv('ACADOS_SOURCE_DIR', acados_root);
setenv('ENV_ACADOS_INSTALL_DIR', acados_root);
% This forces the MEX compiler to look in the acados folder for link_libs.json
setenv('ACADOS_INSTALL_DIR', acados_root); 

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external', 'jsonlab'));
addpath(fullfile(acados_root, 'external', 'casadi-matlab'));

% Project folders
addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

import casadi.*

% --- Setup Model ---
model_name = 'maglev_nmpc';
nx = 12; sym_x = SX.sym('x', nx); 
pos = sym_x(1:3); rot_angles = sym_x(4:6); 
vel = sym_x(7:9); ang_vel = sym_x(10:12);
nu = 4; sym_u = SX.sym('u', nu);
parameters_maggy_V4;

% --- Build Dynamics ---
cx = cos(rot_angles(1)); sx = sin(rot_angles(1));
cy = cos(rot_angles(2)); sy = sin(rot_angles(2));
cz = cos(rot_angles(3)); sz = sin(rot_angles(3));
R_mat = [cy*cz, cz*sx*sy - cx*sz, cx*cz*sy + sx*sz;
         cy*sz, cx*cz + sx*sy*sz, -cz*sx + cx*sy*sz;
         -sy,   cy*sx,            cx*cy];

n_segments = 4; 
theta = linspace(0, 2*pi - 2*pi/n_segments, n_segments);
K_const = -params.magnet.J / params.physical.mu0;
fx_total = 0; fy_total = 0; fz_total = 0;
tx_total = 0; ty_total = 0; tz_total = 0;

for k = 1:n_segments
    p_local = [params.magnet.r * cos(theta(k)); params.magnet.r * sin(theta(k)); 0];
    p_world = R_mat * p_local + pos;
    [bx_k, by_k, bz_k] = casadiComputeFieldBase(p_world(1), p_world(2), p_world(3), sym_u, params);
    tangent_world = R_mat * [cos(theta(k)+pi/2); sin(theta(k)+pi/2); 0];
    dF = cross(K_const * params.magnet.l * tangent_world, [bx_k; by_k; bz_k]);
    segment_weight = (2*pi*params.magnet.r) / n_segments;
    fx_total = fx_total + dF(1) * segment_weight;
    fy_total = fy_total + dF(2) * segment_weight;
    fz_total = fz_total + dF(3) * segment_weight;
    dT = cross(p_world - pos, dF);
    tx_total = tx_total + dT(1) * segment_weight;
    ty_total = ty_total + dT(2) * segment_weight;
    tz_total = tz_total + dT(3) * segment_weight;
end

acc = ([fx_total; fy_total; fz_total] + [0; 0; -params.magnet.m * params.physical.g]) / params.magnet.m;
ang_acc = diag(params.magnet.I) \ ([tx_total; ty_total; tz_total] - cross(ang_vel, diag(params.magnet.I) * ang_vel));
sym_xdot = [vel; ang_vel; acc; ang_acc];

% --- Create Acados OCP ---
ocp = AcadosOcp();
ocp.model.name = model_name;
ocp.model.x = sym_x;
ocp.model.u = sym_u;
ocp.model.f_expl_expr = sym_xdot;

% --- THE FIX: Force External Layout ---
ocp.code_gen_opts.code_export_directory = fullfile(project_root, 'c_generated_code');
% This tells acados NOT to look in the current folder for the lib folder
ocp.code_gen_opts.acados_lib_path = fullfile(acados_root, 'lib');
ocp.code_gen_opts.acados_include_path = fullfile(acados_root, 'include');

% Solver Options
ocp.solver_options.N_horizon = 20;
ocp.solver_options.tf = 1.0;
ocp.solver_options.nlp_solver_type = 'SQP_RTI';
ocp.solver_options.integrator_type = 'ERK';
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';

% --- Cost (Corrected Concatenation) ---
ocp.cost.cost_type = 'LINEAR_LS';
ocp.cost.cost_type_e = 'LINEAR_LS';
ocp.cost.cost_type_0 = 'LINEAR_LS'; 

% Use semicolons to stack these into a single column vector for diag()
Q_vec = [100*ones(3,1); 10*ones(3,1); ones(6,1)];
R_vec = 0.1*ones(4,1);

ocp.cost.W = diag([Q_vec; R_vec]);    % Full 16x16 matrix
ocp.cost.W_e = diag(Q_vec);           % Terminal 12x12 matrix
ocp.cost.W_0 = ocp.cost.W;            % Initial 16x16 matrix

ocp.cost.Vx = zeros(16, 12); ocp.cost.Vx(1:12,1:12) = eye(12);
ocp.cost.Vu = zeros(16, 4);  ocp.cost.Vu(13:16,1:4) = eye(4);
ocp.cost.Vx_e = eye(12);
ocp.cost.Vx_0 = ocp.cost.Vx;
ocp.cost.Vu_0 = ocp.cost.Vu;

ocp.cost.yref = zeros(16, 1);
ocp.cost.yref_e = zeros(12, 1);
ocp.cost.yref_0 = zeros(16, 1);
% Constraints
ocp.constraints.idxbu = 0:3; 
ocp.constraints.lbu = -5 * ones(4, 1); 
ocp.constraints.ubu =  5 * ones(4, 1); 
ocp.constraints.x0 = [0; 0; 0.05; 0; 0; 0; 0; 0; 0; 0; 0; 0]; 

% --- Final Solver Call ---
try
    ocp_solver = AcadosOcpSolver(ocp);
    disp('Success.');
catch e
    rethrow(e);
end

%% --- CLOSED-LOOP SIMULATION ---
N_sim = 100;                 
x_current = ocp.constraints.x0; 
history_x = zeros(nx, N_sim+1);
history_u = zeros(nu, N_sim);
history_x(:,1) = x_current;

fprintf('Starting simulation...\n');

for i = 1:N_sim
    % 1. Set initial state constraint
    ocp_solver.set('constr_x0', x_current);
    
    % 2. Solve the OCP
    ocp_solver.solve();
    
    % 3. Get first control (Stage 0)
    u0 = ocp_solver.get('u', 0);
    
    % 4. Get next state (Predictive step)
    x_next = ocp_solver.get('x', 1); 
    
    % Store history
    history_x(:, i+1) = x_next;
    history_u(:, i) = u0;
    
    % Update for next step
    x_current = x_next;
    
    % 5. SHIFT HORIZON (Warm Start)
    % Use ocp.dims.N to ensure the variable exists
    for j = 0:ocp.dims.N-2
        ocp_solver.set('x', ocp_solver.get('x', j+1), j);
        ocp_solver.set('u', ocp_solver.get('u', j+1), j);
    end
    % Set the last stage to a sensible default (steady state)
    ocp_solver.set('x', ocp_solver.get('x', ocp.dims.N-1), ocp.dims.N-1);
end

disp('Simulation finished.');

%% --- VISUALIZATION ---
figure('Color', 'w', 'Name', 'MagLev NMPC Results');

% Subplot 1: 3D Trajectory
subplot(2, 2, [1, 3]);
plot3(history_x(1,:), history_x(2,:), history_x(3,:), 'b', 'LineWidth', 2);
hold on;
grid on;
plot3(0, 0, 0.05, 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Target Setpoint
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Magnet Center of Mass Trajectory');
axis equal;
view(45, 30);

% Subplot 2: Z-Position (Height) vs Time
subplot(2, 2, 2);
plot(history_x(3,:), 'r', 'LineWidth', 1.5);
hold on; yline(0.05, '--k'); % Setpoint line
title('Vertical Position (Height)');
ylabel('Z [m]');
grid on;

% Subplot 3: Control Inputs (Currents)
subplot(2, 2, 4);
plot(history_u', 'LineWidth', 1.2);
title('Solenoid Currents');
xlabel('Step'); ylabel('Current [A]');
legend('I1', 'I2', 'I3', 'I4');
grid on;

% Add to the visualization section
subplot(2, 2, [1, 3]);
hold on;
% Draw a simple cylinder to represent the magnet at the final position
[xc, yc, zc] = cylinder(params.magnet.r, 20);
zc = zc * params.magnet.l - (params.magnet.l/2); % Center the cylinder
surf(xc + x_current(1), yc + x_current(2), zc + x_current(3), 'FaceColor', 'g', 'EdgeColor', 'none');
camlight; lighting phong;