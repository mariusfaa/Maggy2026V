addpath(genpath("."));
load_params(MaglevModel.Fast);

nx = 12;
nu = length(params.solenoids.r);
ny = 3;

% Setting up system equations
f = @(x,u) maglevSystemDynamics(x,u,params,modelId);
h = @(x,u) maglevSystemMeasurements(x,u,params,modelId);

% Generate dx=f(x,u) as a simulink function

[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(params,modelId);

xLp = [0,0,zEq(1),zeros(1,9)]'; % Linearizing around the equilibria
uLp = zeros(nu,1);

x0 = xLp + [0 0.00025 0.002 0 deg2rad(2) 0 zeros(1, 6)].';
u0 = zeros(nu, 1);

% Linearization
delta = 1e-6; % Step-size used in numerical linearization
[A,B,C,D] = finiteDifferenceLinearization(f,h,xLp,uLp,delta);

% Defining reduced order system
I = [1:5,7:11];
Ared = A(I,I);
Bred = B(I,:);
Cred = C(:,I);
Dred = D(:,:);


% Cost matrices
Q = diag([1e6,1e6,1e3, 1e1,1e1, 1e2,1e2,1e2, 1e2,1e2]);
R = 1e-0*eye(nu);

% Computing LQR estimate
Kred = lqr(Ared,Bred,Q,R);

% increasing order of our controller for controlling the real system
K = [Kred(:,1:5), zeros(4,1), Kred(:,6:end), zeros(4,1)];


%% Generate C code from maglevSystemDynamics
cfg = coder.config('lib');
cfg.GenerateReport = true;
cfg.EnableOpenMP = false;
codegen -config cfg maglevSystemDynamics -args {x0, u0, coder.Constant(params), coder.Constant(modelId)} -report

%% Find generated source files
genDir = fullfile('codegen', 'lib', 'maglevSystemDynamics');
cFiles = dir(fullfile(genDir, '*.c'));
srcFiles = {cFiles.name};

%% Wrap with Legacy Code Tool
% Check maglevSystemDynamics.h for the actual signature — params and modelID
% are baked in as constants, so the generated C function likely only takes
% x and u as inputs and returns dx.
def = legacy_code('initialize');
def.SFunctionName = 'maglevDynamics_sfun';
def.OutputFcnSpec = sprintf('void maglevSystemDynamics(double u1[%d], double u2[%d], double y1[%d])', nx, nu, nx);
def.IncPaths  = {genDir};
def.SrcPaths  = {genDir};
def.SourceFiles = srcFiles;
def.HeaderFiles = {'maglevSystemDynamics.h'};

legacy_code('sfcn_cmex_generate', def);
legacy_code('compile', def);
legacy_code('sfcn_tlc_generate', def);

fprintf('Done. Use S-Function block "maglevDynamics_sfun" in Simulink.\n');
fprintf('Inputs: x [%d x 1], u [%d x 1]\n', nx, nu);
fprintf('Output: dx [%d x 1]\n', nx);