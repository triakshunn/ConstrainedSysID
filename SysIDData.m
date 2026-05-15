%% SysID data script to gather all relevant data for the identification

% Author: Daniel Haugk, 2024, University of Michigan

%% get necessary data for data processing

% set data paths
datapaths = {'go1_leg_data.npz'};  % Go1: .npz file(s) from data collection

% set the start time for each data file that is used
cutTimeBefore = [0];   % set to 0 if data starts at t=0, else set trim time (s)

% set the end time for each data file that is used
cutTimeAfter = [30];   % trim data after this time (s) — adjust to your recording length

% set the wished number of data points
pointNumber = 2100;

% set the cutoff frequency for torque for each data file that is used
cfT = [30];   % torque cutoff frequency (Hz) — tune based on Go1 noise profile

% set the cutoff frequency for velocity for each data file that is used
cfV = [15];   % velocity cutoff frequency (Hz)

% set the cutoff frequency for acceleration for each data file that is used
cfA = [10];   % acceleration cutoff frequency (Hz) — lower than cfV to reduce noise amplification

% set the Butterworth filter order for torque
orderT = 4;

% set the Butterworth filter order for velocity
orderV = 4;

% set the Butterworth filter order for acceleration
orderA = 4;

%% data processing

% this part is robot specific and has to be changed accordingly! 
% specific parts which need to be changed depending on the robot and general parts which don't need to be changed are marked accordingly

% specify the Go1 leg to identify (Go1 specific)
% Options: 'FR' (front-right), 'FL' (front-left), 'RR' (rear-right), 'RL' (rear-left)
leg = 'FR';

% set variables for all data, if more than 1 data file is used (general part)
i = 0;
t = [];
torq = [];
qdot = [];
qddot = [];
q = [];

% get the data for each file seperately and then connect them (general part)
for k = 1:length(datapaths)

    % run step 1 of Go1 file (Go1 specific)
    [time,pos,vel,torque,J,na_idx,nu_idx,~,~,~,~] = SysIDGo1(datapaths{k},leg,1,[],[],[],[],[],constraintVariant);
    
    % process the obtained raw data (general part)
    [i_,t_,q_,qdot_,qddot_,torq_] = dataProcessing(time,pos,vel,torque,cfT(k),cfV(k),cfA(k),cutTimeBefore(k),cutTimeAfter(k),orderT,orderV,orderA,pointNumber);

    % connect the data from each data file (general part)
    i = i + i_;
    t = [t t_];
    torq = [torq torq_];
    qdot = [qdot qdot_];
    qddot = [qddot qddot_];
    q = [q q_];

end

% run step 2 of Go1 file (Go1 specific)
[~,~,~,~,~,~,~,W_ip,T,data,dataFull] = SysIDGo1(datapaths,leg,2,q,qdot,qddot,torq,i,constraintVariant);