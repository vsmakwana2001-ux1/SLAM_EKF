function [particles]= SLAM_PF(particles,vel_cmd,z,index_fov)
global num_landmarks num_particles Q R
%% Predict
for p = 1:num_particles
    particles(p).position = Propagation(particles(p).position,vel_cmd,sqrt(Q)*randn(2,1));  %%SYSTEM DYNAMICS EQUATION
end

%% Update WITH simple data association (nearest neighbour)
doResample = false;

for p = 1:num_particles

    for m = 1:size(z,2)              % loop over each measurement z(:,m)
        if norm(z(:,m)) < 1e-6       % skip empty measurements
            continue;
        end

        %1.Find nearest landmark (data association)
        best_l   = 0;                % index of best landmark
        best_dist = inf;             % distance measure

        for l = 1:num_landmarks
            if ~index_fov(l)         % landmark not in FoV of this car
                continue;
            end

            % predicted measurement for landmark l from this particle
            z_pred = Measurement(particles(p).position, ...
                                 particles(p).landmarks(l).pos);

            innov  = z(:,m) - z_pred;          % measurement difference
            dist   = norm(innov);              % Euclidean distance

            if dist < best_dist
                best_dist = dist;
                best_l    = l;
            end
        end

        %2. Gate and update that landmark only
        if best_l > 0 && best_dist < 5        % 5 m gate (simple threshold)
            l = best_l;

            H = [1 0; 0 1];                   % Jacobian for x–y measurement
            P = particles(p).landmarks(l).P;
            S = H*P*H' + R;
            K = P*H'/S;

            z_pred   = Measurement(particles(p).position, ...
                                   particles(p).landmarks(l).pos);
            residual = z(:,m) - z_pred;

            % EKF update for landmark l of this particle
            particles(p).landmarks(l).pos = ...
                particles(p).landmarks(l).pos + K*residual;
            particles(p).landmarks(l).P   = (eye(2) - K*H)*P;

            % Particle weight update
            particles(p).w = particles(p).w * ...
                (1/sqrt(det(2*pi*S))) * exp(-0.5*residual'/S*residual);

            doResample = true;
        end
    end
end

% Resample all particles based on their weights
if doResample
    particles = Resample(particles);
end

% %% Update
% doResample = false;
% for l = 1:num_landmarks
%     if index_fov(l) % If within field of view
%         doResample = true;        
%         for p = 1:num_particles           
%             z_p = Measurement(particles(p).position,particles(p).landmarks(l).pos);  %%PREDICT MEASUREMENT 
%             residual = z(:,l) - z_p;   %%RESIDUAL = PREDICTED AND TRUE MEASUREMENT DIFFERENCE
% 
%             % Calculate the Kalman gain
% 
%             H =[1 0;0 1];  %%eye(2) %%DOUBT ARE WE REQUIRED TO WRITE WHAT I HAVE WRITTEN OR THE EQUATION
%             P = particles(p).landmarks(l).P;
%             S = H*P*H' + R;
%             K = P*H'/S;
% 
%             % Update EKF
%             % particles(p).landmarks(l).pos = particles(p).landmarks(l).pos + K * residual; 
%             % particles(p).landmarks(l).P = (eye(2) - K * H) * P; 
%             % particles(p).w = particles(p).w * mvnpdf(residual', zeros(2,1), S);
%             particles(p).landmarks(l).pos = particles(p).landmarks(l).pos + K*residual;
%             particles(p).landmarks(l).P = (eye(2) - K*H)*P;  %%IF NOT SURE ABOUT SIZE I=OF I MATRIX eye(size(K,1))
% 
%             % Update particle filter
%             particles(p).w = (particles(p).w)*(1/sqrt(det(2*pi*S)))*exp((-0.5)*residual'/S*residual);
%         end
%     end
% end
% 
% % Resample all particles based on their weights
% if doResample
%     particles = Resample(particles);
% end



