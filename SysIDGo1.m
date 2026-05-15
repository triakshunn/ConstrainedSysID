%% Go1 specific data and functions for sysID

% This script is for a single leg of the Unitree Go1 quadruped.
% The script is split into 2 parts.
% In part 1 (step 1) the raw data is extracted from the .npz data file.
% In part 2 (step 2) the processed data is used to compute the observation
% matrix W_ip, torque vector T, and data matrices.
%
% Key differences from SysIDDigit:
%   - No constraints (open kinematic chain: nc=0, nu=0)
%   - 3 actuated joints per leg (all joints actuated: na=3)
%   - Regressor built using RegressorClassical (in regressor/ folder)
%   - Data loaded from .npz file via Python interface
%
% Outputs (identical signature to SysIDDigit):
%   t_       = raw time vector (1 x N)
%   pos      = raw position matrix (3 x N)  [rad]
%   vel      = raw velocity matrix (3 x N)  [rad/s]
%   torq_    = raw torque matrix   (3 x N)  [Nm]
%   J        = [] (no constraints for Go1)
%   na_idx   = [1 2 3] (all joints actuated)
%   nu_idx   = [] (no unactuated joints)
%   W_ip     = observation matrix for inertial parameters (3*N x 30)
%   T        = torque vector (3*N x 1)
%   data     = [pos vel acc] data matrix (3 x 3*N)
%   dataFull = same as data (no unactuated joints to fill for Go1)
%
% Author: Adapted for Go1 from SysIDDigit.m (Daniel Haugk, 2024, U-Michigan)

function [t_, pos, vel, torq_, J, na_idx, nu_idx, W_ip, T, data, dataFull] = SysIDGo1(datapaths, leg, step, q, qdot, qddot, torq, i, ~)

    %% Go1 leg configuration — open chain, no constraints
    J      = [];        % no constraint Jacobian
    na_idx = [1 2 3];   % all 3 joints are actuated
    nu_idx = [];        % no unactuated joints

    %% Build robot model for regressor computation (Step 2)
    % Requires Go1Leg_model.m and regressor/ folder on MATLAB path
    model = Go1Leg_model(leg);
    model = postProcessModel(model);

    %% STEP 1: Load raw data from .npz file
    if step == 1

        % Unused outputs in step 1
        W_ip     = [];
        T        = [];
        dataFull = [];

        % Load .npz using MATLAB Python interface
        % Expected keys in the .npz file:
        %   'q'   -> (3 x N) joint positions [rad]
        %   'qd'  -> (3 x N) joint velocities [rad/s]
        %   'tau' -> (3 x N) motor torques [Nm]
        %   't'   -> (1 x N) time stamps [s]
        np  = py.importlib.import_module('numpy');
        npz = np.load(datapaths, pyargs('allow_pickle', true));

        q_raw   = double(npz{'q'});
        qd_raw  = double(npz{'qd'});
        tau_raw = double(npz{'tau'});
        t_raw   = double(npz{'t'});

        % Ensure (3 x N) orientation
        if size(q_raw, 1) ~= 3,   q_raw   = q_raw';   end
        if size(qd_raw, 1) ~= 3,  qd_raw  = qd_raw';  end
        if size(tau_raw, 1) ~= 3, tau_raw = tau_raw';  end
        if size(t_raw, 1) > 1,    t_raw   = t_raw';    end

        % Remove duplicate timestamps
        [t_, ia, ~] = unique(t_raw);
        t_ = t_(~isnan(t_));
        ia = ia(~isnan(t_raw(ia)));

        % Extract unique samples
        pos   = q_raw(:, ia);
        vel   = qd_raw(:, ia);
        torq_ = tau_raw(:, ia);

        % Remove last sample (reserved for central difference in dataProcessing)
        pos(:, end)   = [];
        vel(:, end)   = [];
        torq_(:, end) = [];

    %% STEP 2: Build observation matrix W_ip and torque vector T
    elseif step == 2

        % Unused outputs in step 2
        t_    = [];
        torq_ = [];

        % For Go1 there are no unactuated joints to fill in — data is complete
        pos = q;
        vel = qdot;
        acc = qddot;

        % Build W_ip by stacking the regressor at each timestep
        % RegressorClassical returns Y (3 x 30) per timestep
        % W_ip grows to (3*N x 30) after stacking
        W_ip = [];
        for j = 1:i
            [Y_ip, ~] = RegressorClassical(model, pos(:,j), vel(:,j), acc(:,j));
            W_ip = [W_ip; Y_ip];                                            
        end
        W_ip = double(W_ip);

        % Torque vector: (3 x N) -> (3*N x 1), column-major stacking
        T = double(reshape(torq, [3*i, 1]));

        % Data matrix: [pos, vel, acc] each (3 x N), concatenated -> (3 x 3*N)
        % This matches the format expected by SysIDAlg* functions
        data     = double([pos, vel, acc]);
        dataFull = data;    % identical for Go1 (no unactuated joint states)

    end

end
