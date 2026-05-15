function model = Go1Leg_model(leg)
%% Go1Leg_model - spatial_v2 model for a single Go1 leg (3-DOF open chain)
%
% Usage:
%   model = Go1Leg_model('FR')  % Front-right leg (default)
%   model = Go1Leg_model('FL')  % Front-left leg
%   model = Go1Leg_model('RR')  % Rear-right leg
%   model = Go1Leg_model('RL')  % Rear-left leg
%
% The trunk is assumed fixed (mounted on table). Each leg has:
%   Joint 1: Hip Roll   (revolute, axis X)  -> body 1: hip link
%   Joint 2: Hip Pitch  (revolute, axis Y)  -> body 2: thigh link
%   Joint 3: Knee       (revolute, axis Y)  -> body 3: calf+foot link (lumped)
%
% Inertial parameters from go1.urdf (FR leg used as reference).
% The foot (fixed to calf) is lumped into body 3.
%
% Output: model struct compatible with spatial_v2_extended v3 library.
%         Required by RegressorClassical.m to compute the regressor Y.
%
% Author: Adapted for Go1 from CheetahLeg_model.m (spatial_v2_extended v3)

if nargin < 1
    leg = 'FR';  %%%% Does FR is only what you need?
end

%% Gravity
model.gravity = [0; 0; -9.81];   % z-down convention (leg hanging in air)

%% Topology
model.NB     = 3;           % 3 moving bodies (hip, thigh, calf+foot)
model.parent = [0, 1, 2];   % serial chain: trunk->hip->thigh->calf
model.NB_rot = 0;           % no rotor modeling  %%%% URDF has rotor no? 

%% Joint types (from URDF <axis> tag)
% Hip roll: axis x="1 0 0" -> Rx
% Hip pitch: axis x="0 1 0" -> Ry
% Knee:      axis x="0 1 0" -> Ry
model.jtype = {'Rx', 'Ry', 'Ry'};

%% No rotors
model.has_rotor = [0, 0, 0];   %%%% see above comment

%% Joint frame transforms (Xtree{i} = transform from parent frame to joint i frame)
% These come from the URDF <origin xyz="..."> in each <joint> tag.
% xlt([x y z]) creates a 6x6 spatial translation transform.
%
% FR leg joint origins from go1.urdf:
%   1_FR_hip_joint:   xyz="0.1881 -0.04675 0"  (trunk -> hip)    axis x
%   1_FR_thigh_joint: xyz="0 -0.08 0"           (hip -> thigh)    axis y
%   1_FR_calf_joint:  xyz="0 0 -0.213"          (thigh -> calf)   axis y

switch upper(leg)
    case 'FR'
        % Hip joint: trunk origin -> hip frame
        model.Xtree{1} = xlt([0.1881, -0.04675, 0]);
        % Thigh joint: hip frame -> thigh frame
        model.Xtree{2} = xlt([0, -0.08, 0]);
        % Calf joint: thigh frame -> calf frame
        model.Xtree{3} = xlt([0, 0, -0.213]);
                
    case 'FL'
        model.Xtree{1} = xlt([0.1881,  0.04675, 0]);
        model.Xtree{2} = xlt([0,  0.08, 0]);
        model.Xtree{3} = xlt([0, 0, -0.213]);

    case 'RR'
        model.Xtree{1} = xlt([-0.1881, -0.04675, 0]);
        model.Xtree{2} = xlt([0, -0.08, 0]);
        model.Xtree{3} = xlt([0, 0, -0.213]);

    case 'RL'
        model.Xtree{1} = xlt([-0.1881,  0.04675, 0]);
        model.Xtree{2} = xlt([0,  0.08, 0]);
        model.Xtree{3} = xlt([0, 0, -0.213]);

    otherwise
        error('Go1Leg_model: unknown leg "%s". Use FR, FL, RR, or RL.', leg);
end

%% Inertial parameters from go1.urdf
% NOTE: model.I{i} is the 6x6 spatial inertia matrix for body i.
%       Used only for forward dynamics / verification, NOT by the regressor.
%       mcI(mass, CoM_vec, I_3x3) builds the spatial inertia.
%       CoM is expressed in the body's joint frame (frame after joint i).
%
% All legs have identical link inertias (mirrored signs on CoM for FL/RL).

switch upper(leg)
    case {'FR', 'RR'}
        %% Body 1: Hip link (FR/RR — y offset is negative)
        % mass=0.591, CoM=[-0.005657, 0.008752, -0.000102] (URDF link frame)
        m1  = 0.591;
        c1  = [-0.005657, 0.008752, -0.000102];
        I1  = [3.34008405e-4,  1.0826066e-5,  1.290732e-6;
               1.0826066e-5,   6.19101213e-4, -1.643194e-6;
               1.290732e-6,   -1.643194e-6,   4.0057614e-4];

        %% Body 2: Thigh link
        % mass=0.92, CoM=[-0.003342, 0.018054, -0.033451]
        m2  = 0.92;
        c2  = [-0.003342, 0.018054, -0.033451];
        I2  = [4.431760472e-3, -5.7496807e-5, -2.18457134e-4;
              -5.7496807e-5,    4.485671726e-3, -5.72001265e-4;
              -2.18457134e-4,  -5.72001265e-4,  7.40309489e-4];

    case {'FL', 'RL'}
        %% Body 1: Hip link (FL/RL — y offset is positive, mirrored)
        m1  = 0.591;
        c1  = [-0.005657, -0.008752, -0.000102];
        I1  = [3.34008405e-4, -1.0826066e-5,  1.290732e-6;
              -1.0826066e-5,   6.19101213e-4,  1.643194e-6;
               1.290732e-6,    1.643194e-6,   4.0057614e-4];

        %% Body 2: Thigh link (mirrored)
        m2  = 0.92;
        c2  = [-0.003342, -0.018054, -0.033451];
        I2  = [4.431760472e-3,  5.7496807e-5, -2.18457134e-4;
               5.7496807e-5,    4.485671726e-3, 5.72001265e-4;
              -2.18457134e-4,   5.72001265e-4,  7.40309489e-4];
end

%% Body 3: Calf + Foot (LUMPED — foot is rigidly fixed to calf)
% Calf:  mass=0.135862, CoM=[0.006197, 0.001408, -0.116695]
% Foot:  mass=0.06,     CoM=[0, 0, -0.213]  (foot joint xyz in calf frame)
%
% Combined using parallel axis theorem:
m_calf = 0.135862;
c_calf = [0.006197, 0.001408, -0.116695];
I_calf = [1.088793059e-3, -2.55679e-7,  7.117814e-6;
          -2.55679e-7,     1.100428748e-3, 2.077264e-6;
           7.117814e-6,    2.077264e-6,  2.4787446e-5];

m_foot = 0.06;
c_foot = [0, 0, -0.213];  % foot CoM in calf frame (foot joint at -0.213z, foot CoM at origin of foot = 0,0,0 in foot frame)
I_foot_at_foot_origin = 9.6e-6 * eye(3);

% mcI(mass, CoM, I_3x3) converts mass/CoM/inertia into a 6x6 spatial inertia matrix.
% Spatial inertias are additive, so we build each body's spatial inertia separately
% and sum them — this correctly handles the parallel axis theorem automatically.
I3_spatial = mcI(m_calf, c_calf, I_calf) + mcI(m_foot, c_foot, I_foot_at_foot_origin);

%% Assemble spatial inertias
model.I{1} = mcI(m1, c1, I1);
model.I{2} = mcI(m2, c2, I2);
model.I{3} = I3_spatial;   % already combined spatial inertia

%% Store metadata for reference
model.leg  = upper(leg);
model.info = 'Go1 single leg model. Foot lumped into calf (body 3). No rotors.';

end
