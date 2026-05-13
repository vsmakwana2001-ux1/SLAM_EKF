global dt num_landmarks
global Q R Rm P0 num_particles

%% Simulation parameters
dt = 0.5;
timesteps = 100;
time = (1:timesteps)*dt;

% landmarks
block = 10;         % size of the road blocks
width = 3;          % width of roads
num_landmarks = 50; % number of landmarks
real_landmarks = -25+50*rand(2,num_landmarks); % position of landmarks x (m) ; y (m)
for l = 1:num_landmarks                     % landmarks located outside the roads)
    if mod(real_landmarks(1,l),block)<.5*width || mod(real_landmarks(1,l),block)>block-.5*width
        real_landmarks(1,l) = real_landmarks(1,l)+width;
    end
    if mod(real_landmarks(2,l),block)<.5*width || mod(real_landmarks(2,l),block)>block-.5*width
        real_landmarks(2,l) = real_landmarks(2,l)+width;
    end
end

% cars   %%MOVING CLOCKWISE
real_car1 = zeros(3,timesteps);
real_car2 = zeros(3,timesteps);
real_car1(:,1) = [-20; 20; 0];     % x (m) ; y (m) ; rotation (rad)
real_car2(:,1) = [-10; 10; 0];     % x (m) ; y (m) ; rotation (rad)

% real control input
timeinterv = timesteps/4;
V = 30/(timeinterv*dt);                        % v (m/s)
psi_dot = -pi/2/(dt*5);                         % psi_dot (rad/s)
real_vel = [V;0]*ones(1,timesteps);             % v (m/s) ; psi_dot (rad/s)
real_vel(:,  timeinterv-1:  timeinterv+3) = [V/1.3;psi_dot]*ones(1,5);
real_vel(:,2*timeinterv-1:2*timeinterv+3) = [V/1.3;psi_dot]*ones(1,5);
real_vel(:,3*timeinterv-1:3*timeinterv+3) = [V/1.3;psi_dot]*ones(1,5);


% noise settings
process_variance = [0.1^2; 0.01^2];                          % Process covariance (Var(v) (m^2/s^2) ; Var(psi_dot) (rad^2/s^2))
measurement_variance = [1^2; 1^2];                           % Measurement covariance (Var(x) (m^2) ; Var(y) (m^2))

% The field of range of a sensor (can sense a landmark)
max_read_distance = 10;

%% Create trajectory/measurement
% Control input with process noise
vel_cmd1 = real_vel-sqrt(diag(process_variance))*randn(2,timesteps);
vel_cmd2 = real_vel-sqrt(diag(process_variance))*randn(2,timesteps);

% Measurements
meas_landmark1 = zeros(length(measurement_variance),num_landmarks,timesteps);
meas_landmark2 = zeros(length(measurement_variance),num_landmarks,timesteps);
meas_landmark1(:,:,1) = Measurement(real_car1(:,1),real_landmarks+randn(2,1));
meas_landmark2(:,:,1) = Measurement(real_car2(:,1),real_landmarks+randn(2,1));

index_seen = zeros(3,num_landmarks,timesteps); % Index for landmark has been discovered at least once (1: car1, 2: car2, 3: Fusion)
index_fov = zeros(3,num_landmarks,timesteps);  % Index for landmark is close enough (1: car1, 2: car2, 3: Fusion)
for t = 2:timesteps
    % Move the car
    real_car1(:,t) = Propagation(real_car1(:,t-1),real_vel(:,t-1),zeros(2,1));
    real_car2(:,t) = Propagation(real_car2(:,t-1),real_vel(:,t-1),zeros(2,1));
    
    % Get the measurements if the landmark is close enough    
    for l = 1:num_landmarks
        if sqrt(sum((real_car1(1:2,t)-real_landmarks(:,l)).^2))<max_read_distance
            meas_landmark1(:,l,t) = Measurement(real_car1(:,t),real_landmarks(:,l)+randn(2,1));
            index_seen(1,l,t:end) = 1;
            index_fov(1,l,t) = 1;
        end
        if sqrt(sum((real_car2(1:2,t)-real_landmarks(:,l)).^2))<max_read_distance
            meas_landmark2(:,l,t) = Measurement(real_car2(:,t),real_landmarks(:,l)+randn(2,1));
            index_seen(2,l,t:end) = 1;
            index_fov(2,l,t) = 1;
        end
    end
end
index_seen(3,:,:) = index_seen(1,:,:).*index_seen(2,:,:);
index_fov(3,:,:) = index_fov(1,:,:).*index_fov(2,:,:);

%% EKF parameters
Q = diag(process_variance);                                   % State covariance
R = diag(measurement_variance);                             % Measurement covariance
Rm = diag([measurement_variance;measurement_variance]);     % Measurement covariance for measurement fusion %%ONLY FOR MEASUREMENT FUSION
P0 = diag([0.1 0.1]);

%% Particle filter parameters
num_particles = 200;   %%YOU CAN PLAY WITH THE NUMBER OF PARTICLE

%% Initial estimates
initial_landmarks = real_landmarks + [1;1].*randn(2,num_landmarks); 
initial_car1 = real_car1(:,1) + [1;1;0].*randn(3,1); 
initial_car2 = real_car2(:,1) + [1;1;0].*randn(3,1); 
             



