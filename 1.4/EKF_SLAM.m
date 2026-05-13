function [x, P] = EKF_SLAM(x, P, vel_cmd, z, index_fov)
% EKF_SLAM - Extended Kalman Filter for SLAM with range-bearing measurements
%
% Inputs:
%   x         : state vector (3 + 2*num_landmarks) x 1 [pose; landmarks]
%   P         : covariance matrix (3+2N) x (3+2N)
%   vel_cmd   : control input [v; omega] (2x1)
%   z         : measurement matrix (2 x num_landmarks) with columns [r; phi]
%   index_fov : logical array (1 x num_landmarks) indicating observed landmarks
%
% Outputs:
%   x         : updated state vector
%   P         : updated covariance matrix
%
% Global variables (set in Initialisation_scenario.m):
%   dt            : time step
%   num_landmarks : total number of landmarks
%   Q             : process noise covariance (2x2) for [v; omega]
%   R             : measurement noise covariance (2x2) for [r; phi]

global dt num_landmarks Q R

% ------------------------------------------------------------------------
% EKF PREDICTION STEP (Motion Update)quarc_sine_scope_demo
% ------------------------------------------------------------------------

% 1. Extract current robot pose from augmented state
x_r = x(1:3);  % [x; y; theta]
theta = x_r(3);
v = vel_cmd(1);
omega = vel_cmd(2);

% 2. STATE PREDICTION: Propagate pose 
%    x_r(k+1|k) = f(x_r(k), u_k) = [x + v*dt*cos(theta); y + v*dt*sin(theta); theta + omega*dt]
x_r_new = x_r + [v * dt * cos(theta);
                 v * dt * sin(theta);
                 omega * dt];
x_new = [x_r_new; x(4:end)];   % Landmarks unchanged during prediction

% 3. JACOBIAN w.r.t. STATE: F_r = df/dx_r (pose block), block-diagonal for landmarks
F_r = [1, 0, -v * dt * sin(theta);
       0, 1,  v * dt * cos(theta);
       0, 0, 1];
F = blkdiag(F_r, eye(2 * num_landmarks));

% 4. JACOBIAN w.r.t. CONTROL NOISE: F_u = df/du to propagate Q to pose space
F_u = [dt * cos(theta), 0;
       dt * sin(theta), 0;
       0,              dt];

% 5. PROCESS NOISE COVARIANCE in pose space: Q_pose = F_u * Q * F_u^T
Q_pose = F_u * Q * F_u';

% 6. AUGMENT NOISE to full state: G maps pose noise to augmented state
G = [eye(3); zeros(2 * num_landmarks, 3)];
Q_full = G * Q_pose * G';

% 7. PREDICTED COVARIANCE: P(k+1|k) = F * P(k) * F' + G * Q * G'
P_new = F * P * F' + Q_full;

% ------------------------------------------------------------------------
% EKF UPDATE STEP (Measurement Update) - Sequential per observed landmark
% ------------------------------------------------------------------------
for l = 1:num_landmarks
    if index_fov(l)
        % 1. Extract landmark position from predicted state
        idx_lx = 3 + 2 * (l - 1) + 1;
        idx_ly = idx_lx + 1;
        lx = x_new(idx_lx);
        ly = x_new(idx_ly);
        
        % 2. PREDICTED MEASUREMENT: z_hat = h(x(k+1|k)) = [r; phi]
        dx = lx - x_new(1);
        dy = ly - x_new(2);
        r = sqrt(dx^2 + dy^2);
        phi = atan2(dy, dx);
        z_hat = [r; phi];
        
        % 3. MEASUREMENT RESIDUAL (INNOVATION): nu = z - z_hat, wrap bearing to [-pi, pi]
        nu = z(:, l) - z_hat;
        nu(2) = wrapToPi(nu(2));
        
        % 4. OBSERVATION JACOBIAN H: dh/dx (sparse: robot pose + this landmark)
        H = zeros(2, length(x_new));
        % Robot pose block
        H(1, 1:3) = [-dx/r, -dy/r, 0];
        H(2, 1:3) = [ dy/r^2, -dx/r^2, 0];
        % Landmark block
        H(1, idx_lx:idx_ly) = [ dx/r,  dy/r];
        H(2, idx_lx:idx_ly) = [-dy/r^2,  dx/r^2];
        
        % 5. INNOVATION COVARIANCE: S = H * P(k+1|k) * H' + R
        S = H * P_new * H' + R;
        
        % 6. KALMAN GAIN: K = P(k+1|k) * H' * inv(S)
        K = P_new * H' / S;   
        
        % 7. STATE UPDATE: x(k+1|k+1) = x(k+1|k) + K * nu
        x_new = x_new + K * nu;
        
        % 8. COVARIANCE UPDATE: P(k+1|k+1) = (I - K*H) * P(k+1|k) (Joseph form optional)
        P_new = (eye(size(P_new)) - K * H) * P_new;
    end
end

% Output updated estimates
x = x_new;
P = P_new;
end
