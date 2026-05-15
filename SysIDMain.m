%% SysID Main Script — Go1 Single Leg

% Author: Daniel Haugk, 2024, University of Michigan
% Adapted for Unitree Go1 by: Akshunn, 2026

% Add paths for Go1-specific files
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'regressor')));

%% set algorithm options

% error filter value for weights
k = 0.3;

% stopping value for while loops
tol = 0.0001;

% number of starting points for first multi search optimization
MS1 = 3;

% number of starting points for second (friction) multi search optimization
MS2 = 3;

% include regrouping or not
% true: reduces 30 params to identifiable subset (recommended for Go1)
regroup = true;

% search for all parameters (inertial, friction, motor, offset) or not
% true: identify inertial + friction + motor (per implementation plan)
SearchAll = true;

% search for friction motor and offset parameters or not
% irrelevant when SearchAll = true
SearchFM = false;

% optimize over friction exponents or not
SearchAlpha = false;

% include offset parameter or not
includeOffset = false;

% include friction and motor dynamics or not (only needed when SearchFM is false)
includeFMDynamics = true;

% Go1 leg is an open kinematic chain — NO constraints needed
includeConstraints = false;

% include normalization and weighting (GLS — recommended)
includeWeighting = true;

% constraint variant: irrelevant when includeConstraints=false, keep for compatibility
constraintVariant = 1;

% fmincon options
options = optimoptions('fmincon','MaxFunctionEvaluations',30000,'Display','iter','Algorithm','sqp','SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,'UseParallel',true);

% collect algorithm options in a struct
AlgOptions = {k,tol,MS1,MS2,regroup,SearchAll,SearchFM,SearchAlpha,includeOffset,includeFMDynamics,includeConstraints,constraintVariant,includeWeighting};


%% set initial conditions — warm-started from go1.urdf
% Structure per link: [Ixx, Ixy, Ixz, Iyy, Iyz, Izz, hx, hy, hz, m]
% hx = m*cx, hy = m*cy, hz = m*cz  (first moments, NOT CoM directly)

X0_ip = [
    % Body 1: Hip link (m=0.591 kg)
    3.340e-4,  1.083e-5,  1.291e-6,  6.191e-4, -1.643e-6,  4.006e-4, ...
    -0.0057*0.591,  0.0088*0.591, -0.0001*0.591,  0.591, ...
    % Body 2: Thigh link (m=0.92 kg)
    4.432e-3, -5.750e-5, -2.185e-4,  4.486e-3, -5.720e-4,  7.403e-4, ...
    -0.0033*0.92,   0.0181*0.92,  -0.0335*0.92,   0.92, ...
    % Body 3: Calf+foot link (m=0.196 kg, combined)
    1.089e-3, -2.557e-7,  7.118e-6,  1.100e-3,  2.077e-6,  2.479e-5, ...
     0.0062*0.196,  0.0014*0.196, -0.1167*0.196,  0.196  ...
]';

% True inertial parameters (unknown — leave empty, optimizer will find them)
X_ip = [];

% Initial condition for motor, friction parameters
% Structure: [Im_1, Im_2, Im_3,  Fc_1, Fv_1,  Fc_2, Fv_2,  Fc_3, Fv_3]
% Im_j: motor inertia reflected to joint (rotor_inertia x gear_ratio^2)
% Start conservatively: Im from rotor mass, Fc/Fv from typical values
X0_fm = [
    1.12e-4,  1.12e-4,  1.12e-4, ...   % Im_1, Im_2, Im_3 (rotor inertia, N=1 assumed)
    0.5, 0.1, ...                        % Fc_1, Fv_1 (hip roll)
    0.5, 0.1, ...                        % Fc_2, Fv_2 (hip pitch)
    0.5, 0.1  ...                        % Fc_3, Fv_3 (knee)
]';

% True friction/motor parameters (unknown — leave empty)
X_fm = [];

% full initial condition vector
X0_1 = [X0_ip;X0_fm];

%%  set lower and upper bounds
% Same structure as X0_ip / X0_fm

% Lower bounds for inertial parameters
% Physical constraints: masses > 0, diagonal inertias > 0, off-diagonal unconstrained
% Using ~0 lower bounds (LMI constraint in fmincon enforces physical feasibility)
lb_ip = [
    % Ixx    Ixy      Ixz      Iyy    Iyz      Izz    hx      hy      hz      m
    1e-6, -1e-2, -1e-2,  1e-6, -1e-2,  1e-6, -1.0,  -1.0,  -1.0,  0.01, ...  % hip
    1e-6, -1e-2, -1e-2,  1e-6, -1e-2,  1e-6, -1.0,  -1.0,  -1.0,  0.01, ...  % thigh
    1e-6, -1e-2, -1e-2,  1e-6, -1e-2,  1e-6, -1.0,  -1.0,  -1.0,  0.01  ...  % calf+foot
]';

% Lower bounds for friction/motor: all non-negative physically
lb_fm = [0, 0, 0,  0, 0,  0, 0,  0, 0]';

% Upper bounds for inertial parameters (URDF values x 5 — generous)
ub_ip = [
    % Ixx     Ixy    Ixz    Iyy     Iyz    Izz    hx    hy    hz    m
    1e-2,  1e-2,  1e-2,  1e-2,  1e-2,  1e-2,  1.0,  1.0,  1.0,  3.0, ...  % hip
    1e-1,  1e-2,  1e-2,  1e-1,  1e-2,  1e-2,  1.0,  1.0,  1.0,  5.0, ...  % thigh
    1e-1,  1e-2,  1e-2,  1e-1,  1e-2,  1e-2,  1.0,  1.0,  1.0,  2.0  ...  % calf+foot
]';

% Upper bounds for friction/motor parameters
ub_fm = [0.1, 0.1, 0.1,  5.0, 2.0,  5.0, 2.0,  5.0, 2.0]';

% Friction exponent bounds (not used: SearchAlpha=false)
lba = []';
uba = []';

% collect bounds in one vector
lb = [lb_ip; lb_fm];
ub = [ub_ip; ub_fm];

%% get data

% this script can be run by itself after setting the algorithm options
run('SysIDData.m');

%% run the system identification

[X, Wfull, alphanew, Ginv] = SysIDAlg(AlgOptions,na_idx,nu_idx,J,lb,ub,lba,uba,T,data,dataFull,W_ip,X0_1,X_ip,X_fm,options);
