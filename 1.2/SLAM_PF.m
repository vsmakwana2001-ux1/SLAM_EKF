function [particles]= SLAM_PF(particles,vel_cmd,z,index_fov)
global num_landmarks num_particles Q R
%% Predict
for p = 1:num_particles
    particles(p).position = Propagation(particles(p).position,vel_cmd,sqrt(Q)*randn(2,1));
end

%% Update
doResample = false;
for l = 1:num_landmarks
    if index_fov(l) % If within field of view
        doResample = true;        
        for p = 1:num_particles           
            z_p = Measurement(particles(p).position,particles(p).landmarks(l).pos);
            residual = z(1,l) - z_p;
            residual = wrapToPi(residual);% Keep bearing within [-π, π]
               % Compute range and bearing for Jacobian calculation
             dx = particles(p).landmarks(l).pos(1) - particles(p).position(1);
             dy = particles(p).landmarks(l).pos(2) - particles(p).position(2);
             r2 = dx^2 + dy^2; % Range  

             % Compute Jacobian H
             H = [-dy/r2, dx/r2];
            % Extract covariance of the landmark
            P = particles(p).landmarks(l).P;
            % Innovation covariance S
            S = H*P*H' + R;
            % Kalman gain calculation
            K = P*H'/S;

            % Update EKF
            particles(p).landmarks(l).pos = particles(p).landmarks(l).pos + K * residual ;
            particles(p).landmarks(l).P = (eye(2) - K * H) * P;

            % Update particle filter
            particles(p).w = particles(p).w * (1 / sqrt(det(2 * pi * S))) * exp(-0.5 * residual' / S * residual) ;
        end
    end
end

% Resample all particles based on their weights
if doResample
    particles = Resample(particles);
end



