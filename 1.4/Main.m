clc; clear all; close all; warning off;
%% Simulation Settings
fusion_flag = 0;            % 0: no fusion / 1: state fusion / 2: measurement fusion / 3: covariance intersection

rng(2025)
Initialisation_scenario;

% Declare globals for plotting and analysis (these are now defined in the base workspace)
global real_landmarks real_car1 real_car2 ...
       meas_landmark1 meas_landmark2 ...
       index_seen index_fov ...
       pos_landmark1 pos_landmark2 pos_landmark_SF ...
       cov_landmark1 cov_landmark2 cov_landmark_SF ...
       pos_car1 pos_car2 max_read_distance block width fusion_flag

% ------------------------------------------------------------------------
% EKF Initialisation
% ------------------------------------------------------------------------
x_car1 = [initial_car1; initial_landmarks(:)];
x_car2 = [initial_car2; initial_landmarks(:)];

P_robot = diag([1, 1, 0.1^2]);
P_landmark = P0;

% Correct block‑diagonal construction
landmark_blocks = repmat({P_landmark}, num_landmarks, 1);
P_car1 = blkdiag(P_robot, landmark_blocks{:});
P_car2 = blkdiag(P_robot, landmark_blocks{:});

x_SF = x_car1;
P_SF = P_car1;

% Preallocate storage
pos_car1 = zeros(3, timesteps);
pos_car2 = zeros(3, timesteps);
pos_landmark1 = zeros(2, num_landmarks, timesteps);
pos_landmark2 = zeros(2, num_landmarks, timesteps);
pos_landmark_SF = zeros(2, num_landmarks, timesteps);
cov_landmark1 = zeros(2, 2, num_landmarks, timesteps);
cov_landmark2 = zeros(2, 2, num_landmarks, timesteps);
cov_landmark_SF = zeros(2, 2, num_landmarks, timesteps);

% Store initial values
pos_car1(:,1) = x_car1(1:3);
pos_car2(:,1) = x_car2(1:3);
pos_landmark1(:,:,1) = reshape(x_car1(4:end), 2, num_landmarks);
pos_landmark2(:,:,1) = reshape(x_car2(4:end), 2, num_landmarks);
pos_landmark_SF(:,:,1) = reshape(x_SF(4:end), 2, num_landmarks);
for l = 1:num_landmarks
    cov_landmark1(:,:,l,1) = P_car1(3+2*l-1:3+2*l, 3+2*l-1:3+2*l);
    cov_landmark2(:,:,l,1) = P_car2(3+2*l-1:3+2*l, 3+2*l-1:3+2*l);
    cov_landmark_SF(:,:,l,1) = P_SF(3+2*l-1:3+2*l, 3+2*l-1:3+2*l);
end

%% Simulation
for t = 2:timesteps
    [x_car1, P_car1] = EKF_SLAM(x_car1, P_car1, vel_cmd1(:,t-1), ...
                                 meas_landmark1(:,:,t), index_fov(1,:,t));
    [x_car2, P_car2] = EKF_SLAM(x_car2, P_car2, vel_cmd2(:,t-1), ...
                                 meas_landmark2(:,:,t), index_fov(2,:,t));
    
    % Fusion placeholder
    switch fusion_flag
        case {1,2,3}
            x_SF = x_car1;
            P_SF = P_car1;
        otherwise
            x_SF = x_car1;
            P_SF = P_car1;
    end

    % Save results
    pos_car1(:,t) = x_car1(1:3);
    pos_car2(:,t) = x_car2(1:3);
    pos_landmark1(:,:,t) = reshape(x_car1(4:end), 2, num_landmarks);
    pos_landmark2(:,:,t) = reshape(x_car2(4:end), 2, num_landmarks);
    pos_landmark_SF(:,:,t) = reshape(x_SF(4:end), 2, num_landmarks);
    for l = 1:num_landmarks
        cov_landmark1(:,:,l,t) = P_car1(3+2*l-1:3+2*l, 3+2*l-1:3+2*l);
        cov_landmark2(:,:,l,t) = P_car2(3+2*l-1:3+2*l, 3+2*l-1:3+2*l);
        cov_landmark_SF(:,:,l,t) = P_SF(3+2*l-1:3+2*l, 3+2*l-1:3+2*l);
    end

    %% Plot – now passing current time t
    Plot_animation(t);
    drawnow;
end
Plot_analysis;